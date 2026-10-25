# Local CDC patches and findings

Notes on the fdcanusb text (CDC) interface. Written while chasing missing replies at 250–500 Hz with 6 motors on one bus.

| | Status |
|---|---|
| ZLP fix | Committed on `master` as `a36284f`. Flashed and **confirmed on hardware**: no more held-back replies. |
| `0002-cdc-tx-ring.patch` | Not applied. Compile-checked only, **not tested on hardware**. Applies on top of `a36284f`. |

Line numbers below refer to upstream `91c2b7f` (tag `0.1-20261006`) unless noted otherwise.

## 1. ZLP fix (`a36284f`)

### The bug

From time to time the last 64 bytes of a burst of output never reached the host. They showed up only when the adapter sent something else.

- **Regression:** upstream commit `07894b9` ("Only report data back to the host when we actually have it", releases `0.1-20260216` and later).
- **Old behaviour:** before that commit, `cdc_txonly` re-armed the IN endpoint with a zero-length packet (ZLP) whenever it had nothing to send. That accidentally ended every transfer properly.
- **What goes wrong now:** if the last packet of a burst is exactly 64 bytes (`wMaxPacketSize`), the endpoint simply goes idle. Linux `cdc_acm` reads 128 bytes at a time, so a lone full 64-byte packet does not complete the read. Those 64 bytes stay in the kernel until the next packet arrives.

The same commit caused a second symptom:

- `IsCdcActive()` only counts IN completions within the last 1 s (`fw/stm32g4_async_usb_cdc.cc:417-422`).
- CAN frames received while it returns false are dropped from CDC (`fw/can_manager.cc:351-355`).
- An idle port no longer produces IN completions. So after more than 1 s of idle, the first reply or two after opening the port were lost.

### The fix

`fw/stm32g4_async_usb_cdc.cc`:

1. Remember the size of the last packet sent. If it was 64 bytes and nothing else is queued, send a ZLP.
2. Mark the endpoint busy while the SET_CONFIGURATION ZLP is still waiting. Otherwise `ProcessWrite` may write to a busy endpoint: `usbd_ep_write` returns -1 and the data is lost.
3. Count OUT packets (host writes) as CDC activity, too.

```diff
@@ -462,6 +462,7 @@ class Stm32G4AsyncUsbCdc::Impl {
       // If the TX endpoint is idle and we now have data, arm it.
       if (tx_idle_ && buffer_.size > 0) {
         tx_idle_ = false;
+        last_tx_size_ = buffer_.size;
         usbd_ep_write(&udev_, CDC_TXD_EP, &buffer_.buf[0], buffer_.size);
         buffer_.size = 0;
       }
@@ -492,6 +493,10 @@ class Stm32G4AsyncUsbCdc::Impl {
         usbd_ep_config(&udev_, CDC_NTF_EP, USB_EPTYPE_INTERRUPT, CDC_NTF_SZ);
         usbd_reg_endpoint(&udev_, CDC_RXD_EP, g_cdc_rxonly);
         usbd_reg_endpoint(&udev_, CDC_TXD_EP, g_cdc_txonly);
+        // The endpoint is busy until this ZLP is collected, so
+        // ProcessWrite must not try to arm it in the meantime.
+        tx_idle_ = false;
+        last_tx_size_ = 0;
         usbd_ep_write(&udev_, CDC_TXD_EP, 0, 0);
 
         // configure gs_usb endpoints
@@ -598,6 +603,10 @@ class Stm32G4AsyncUsbCdc::Impl {
     const auto actual = usbd_ep_read(&udev_, ep, &fifo_[fpos_], bytes_to_read);
     fpos_ += actual;
 
+    // The host writing to us shows the port is open just as well as
+    // it reading from us does.
+    last_cdc_activity_us_ = options_.timer->read_us();
+
     led_com_.write(1);
 
     ProcessRead();
@@ -623,6 +632,14 @@ class Stm32G4AsyncUsbCdc::Impl {
     last_cdc_activity_us_ = options_.timer->read_us();
 
     if (buffer_.size == 0) {
+      if (last_tx_size_ == CDC_DATA_SZ) {
+        // The packet just sent was full size, so the host cannot
+        // tell the transfer has ended.  Terminate it with a ZLP.
+        last_tx_size_ = 0;
+        usbd_ep_write(&udev_, ep, &buffer_.buf[0], 0);
+        return;
+      }
+
       // We have no data to send.  Leave the endpoint in the NAK state
       // so the hardware automatically NAKs IN tokens without
       // generating further callbacks.  ProcessWrite will re-arm the
@@ -633,6 +650,7 @@ class Stm32G4AsyncUsbCdc::Impl {
 
     led_com_.write(1);
 
+    last_tx_size_ = buffer_.size;
     usbd_ep_write(&udev_, ep, &buffer_.buf[0], buffer_.size);
     buffer_.size = 0;
   }
@@ -693,6 +711,8 @@ private:
 
   Buffer buffer_;
   bool tx_idle_ = true;
+  // Size of the packet most recently handed to the TX endpoint.
+  size_t last_tx_size_ = 0;
 
   micro::SizeCallback current_read_callback_;
   mjlib::base::string_span current_read_data_;
```

## 2. Round-trip time: where it goes

Measured on the host: a batch of 6 `can send` lines (requests with 29-bit IDs and 12–16 B payloads) gets its 6 `rcv` replies (11-bit IDs, 12 B) back in about 1.1–1.2 ms.

### The CAN bus is the floor

At the defaults (`can.bitrate` 1 M, `can.fd_bitrate` 5 M, BRS on), 6 requests plus 6 replies take **about 0.85–0.9 ms of pure bus time**. The estimate per frame:

```
t_frame ≈ (A + 14)·(1+s)/R_nom + (33 + 8·P)·(1+s)/R_data
  A = 17 (11-bit ID) or 36 (29-bit ID)
  P = payload rounded up to a valid DLC (13..15 -> 16)
  s ≈ 0.1 (bit stuffing)
```

Roughly: request ≈ 82–89 µs, reply ≈ 59 µs.

About 60 % of those bits go at the 1 Mbit/s nominal rate, so a faster data rate helps only a little:

| Change | Bus time per batch |
|---|---|
| `fd_bitrate` 8 Mbit/s | ≈ 0.74 ms |
| Two buses, 3 motors each | ≈ 0.45 ms per bus |

The firmware adds only about 0.2–0.3 ms on top of the bus floor.

### The firmware overhead: parser stalls behind USB output

1. The command parser (mjlib `CommandManager`) reads the next line only after the previous reply has been accepted by the shared write stream.
2. `CanManager::EmitData` holds that stream until its whole `rcv` buffer has been accepted (`fw/can_manager.cc:732-753`).
3. The CDC driver accepts only what fits in its single 64-byte `buffer_` (`fw/stm32g4_async_usb_cdc.cc:443-471`). The rest waits for IN packets to complete.
4. So once the first reply comes in, the remaining `can send` lines are parsed late, and the bus sits idle in the meantime. On the host this shows up as `OK` #5 and #6 arriving only together with the `rcv` lines.

Each IN/OUT packet also seems slow on the host side, about 150–230 µs. The endpoints are single-buffered and serviced from the main loop, so they NAK between packets. How quickly the host retries after a NAK depends on the USB controller. A USB hub in between makes it slower, so check `lsusb -t`.

## 3. `0002-cdc-tx-ring.patch` (next step, untested)

**What it does:** replaces the 64-byte CDC TX `buffer_` with a 512-byte ring buffer.

- The ring lives in static `.bss`, not in the 20 KB `micro::SizedPool`. The pool is nearly full, and an overflow asserts at boot.
- `ProcessWrite` accepts as much as fits.
- Each IN packet carries `min(64, queued)` bytes, through a 64-byte staging buffer so that wrap-around is handled.
- The ZLP logic from `a36284f` is kept: a full packet followed by an empty ring still gets a ZLP.

**Why it should help:**

- A whole cycle of output (about 306 B) is accepted at once, so the parser never waits and the requests go out at bus speed.
- Packets are full while a backlog exists, so there are fewer of them.
- Expected round trip about **0.95–1.05 ms** (from 1.1–1.2 ms).

**Cost and risk:**

- About +128 B flash, +512 B RAM.
- No change to the protocol or output format; gs_usb is not touched.
- Risk is low, but it has not run on hardware yet.

**Apply and test:**

```bash
git apply patches/0002-cdc-tx-ring.patch
./tools/bazel build //fw:fdcanusb
./flash.py              # no --erase, that wipes the saved conf
```

Compare the host-side batch round-trip time with the ZLP-only firmware (`a36284f`). If you have the HIL rig, run `tests/test_hardware_in_loop.py` too, in particular:

- `test_dut_cdc_to_legacy_cdc`
- `test_bidirectional_simultaneous`
- `test_large_frames` (exercises ring wrap-around)
- `test_cdc_overflow`

## 4. Further ideas (not saved as patches)

Ordered by value/risk. Except for double buffering, all were prototyped and compile-checked once, but not kept.

- **Never drop host input:** today `cdc_rxonly` silently drops OUT bytes that don't fit in the 512-byte input FIFO (`fw/stm32g4_async_usb_cdc.cc:597-598`). Better: leave the packet in the endpoint, which NAKs the host, until there is room. This matters when the host pipelines several cycles ahead.
- **More work per main-loop pass:** `CanManager::Poll` moves only one queued frame into the 3-slot hardware TX FIFO, and takes only one RX frame, per pass (`fw/can_manager.cc:290-310, 329`). Filling and draining both completely costs nothing and gives more RX headroom.
- **Shorter output:**
  - an option to drop the ` e B F r x-1` fields from `rcv` lines (47 → 35 B);
  - a `Q` send flag that suppresses `OK`.
  - Together these cut output per cycle from 306 to 210 B.
  - Put new settings in a **new** persistent group (e.g. `cdc`). Adding a field to the `can` group changes its schema CRC, and mjlib then silently discards the saved `can` settings on the next boot (mjlib `micro/persistent_config.cc:366-372`).
- **Double-buffered USB endpoints:** this would remove the NAK between packets. However, the libusb_stm32 driver's double-buffered IN path looks broken as written: `ep_config` sets `STAT_TX` to VALID, but `ep_write` only accepts the NAK state (`fw/usbd_stm32g474_devfs.c:34, 224, 350`). It needs driver work and hardware testing. Only worth it if packets are still slow after `0002`.
- **Latent issues:**
  - a full 32-entry TX queue silently drops the frame but still replies `OK` (`fw/can_manager.cc:669`);
  - RX FIFO overflows are not reported anywhere (FDCAN `RF0L` could be shown in `can status`).
- **Not worth it:**
  - RX by interrupt (polling is fast enough);
  - making mjlib's command replies non-blocking (the ring buffer achieves the same).
