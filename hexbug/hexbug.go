/*
# hexbug.go

hexbug.go implements an irtrx.RxStateMachine that decodes the IR signals from a 6-button, 4-channel HEXBUG BattleBots remote control.

I found two other projects of people analyzing the hexbug protocol, but they were a mix of wrong and also from the perspective of controlling a hexbug,
rather than receiving commands. I have lots of little robot-like toys that could easily be made RC with a ~$3 MCU, a ~$1 dual H-Bridge, and a ~$2 IR receiver.

## Hardware

I have not directly measured the frequency of the IR transmitter, but a 38kHz demodulating IR receiver works.

You should add in a couple of capacitors and resistors as the datasheet recommends, but if you're lazy, cheap, or, like me, both, it'll still work fine.

## Protocol

It seems standard for these 38kHz IR receivers to idle high--that is, there is a pull-up resistor keeping the idle line value at VCC (5 Volts).
When an IR signal is detected, it pulls the line low. Thus, an "on" signal the output pin reads low (ground) and off reads high (VCC).

Because of this inversion, I have confusingly decided to call idle "on" time and active "off" time.

As far as I can tell through use of a logic analyzer and some debugging programs running on an rp2040, the "on" times don't matter (except as spacers),
it's all about the off times. So when handling an On-Off pair. We really only concern ourselves with the Off times.

## Data Structure

We get a start flag that has an off time of about 1.7ms; this indicates the start of a byte. The rest of the on/off pairs are bits. The On time for bits
is between 300 and 400us. Both logic analyzer and rp2040 measurements show there to be a large variance, about +/- 14%. But this might be because I have
no supporting circuitry (caps and resistors) as recommended to smooth out power to the receiver.

But the signalling is fairly robust; any off time longer that 1.4ms is a start flag; between 750us and 1.3ms is a one and less than 750us (typically around
350us) is a zero.

I have decoded the bits as LSB, but it largely doesn't matter.

After the start flag, we get 9 bits. The first 6 correspond to the 6 buttons on the controller,

	| Button  | Val |  Hex |
	|^^^^^^^^^|^^^^^|^^^^^^|
	|      Fwd|   1 | 0x01 |
	|     Back|   2 | 0x02 |
	|     Left|   4 | 0x04 |
	|    Right|   8 | 0x08 |
	| LeftWeap|  16 | 0x10 |
	|RightWeap|  32 | 0x20 |

The next two bits are the channel. Taken as a whole integer, Hex gives the int value; 2-bit val is looking at just the relevant bits.

	| Ch |  Hex | 2-bit Val |
	^^^^^^^^^^^^^^^^^^^^^^^^^
	|  1 | 0x00 | 0         |
	|  2 | 0x40 | 1         |
	|  3 | 0xC0 | 3         |
	|  4 | 0x80 | 2         |

Yes, channel 3 and 4 seem like they've been swapped.

The final bit sent is an odd parity bit. That is, this bit is used to ensure the total number of 1s is an odd number.

This package reads in the bits LSB first, thus the 9 bits look this, with F being the first bit received and P being the ninth.

	PCCUDRLBF

	P - Parity
	C - Channel bit
	U - Right Weapon button
	D - Left Weapon Button
	R - Right button
	L - Left Button
	B - Back Button
	F - Forward Button

## What the Transmitter Sends

Upon pressing a button, the transmitter sends 2 bytes indicating the button(s) pressed. If the button is held down, it repeats the same byte until released.
It might actually only send one, and this is a timing/debounce issue on my part; but it makes sense to me to at a minimum double up on a solitary press.
There is about 6ms between the initial two bytes sent.

When a button is released, 10 stop bytes are sent--stop bytes are just a regular byte, but with all the button bits as zero. For this automatic send of
stop bits, there's about 200ms between each byte.

Pressing multiple buttons at the same time causes a byte with the corresponding bits set to be sent. The controller's buttons are physically made so that
you can't press both forward and backward at the same time, or left and right at the same time. But the protocol can handle all the buttons being pressed
at once.

When you swap channels, the transmitter sends a bunch of stop bytes on the new channel.

## Implementation Notes

## Physical Testing

In direct sunlight, I got surprisingly good results--roughly 2 meters of workable range.

I don't have a big enough space to actually find the indoor range limit, but I got over 6 meters of workable range and there's even decent
non-line-of-sight functionality. Slightly around a corner doesn't seem to be a problem. Totally acceptable for low-speed, lightweight ground
robots.

## Examples

### An example that dumps the bytes received
```

	// gpio pin connected to output of demodulating IR receiver
	const rxPin = machine.GPIOX
	// create the hexbug StateMachine with a callback func that prints
	// the bits of the received byte. (on RP2040 devices, this defaults to usb UART)
	hb := hexbug.NewStateMachine(func(cmd int16) {
		fmt.Printf("%09b\r\n", cmd)
	})
	// create the irrx Rx device; configures rxPin as an input
	rx := irtrx.NewRxDevice(rxPin, hb)
	// set the interrupt handler
	rx.Start()

	// Keep the mcu in sleeping/low power state; interrupt handler takes care of everything
	for {
		device.Arm("wfi")
	}

```

### Automatic Channel Assignment

Here's a simple callback, suitable for passing to hexbug.NewStateMachine(), that takes the first received channel as its own, like how hexbugs themselves work.

A separate control loop checks r.Cmd and responds to it.

```

	func (r *robot) OnCmd(cmd int16) {
		addr := cmd & irrx.HexbugChMask
		if r.Id == -1 {
			// don't have an id set? take the first one we encounter
			r.Id = addr
		}
		if addr != r.Id {
			// not us?
			return
		}
		r.Cmd = cmd
	}

```
*/
package hexbug

import (
	"time"

	"github.com/sparques/irtrx"
)

const (
	CmdStop          = 0
	CmdFwdMask       = 0b000000001
	CmdBackMask      = 0b000000010
	CmdLeftMask      = 0b000000100
	CmdRightMask     = 0b000001000
	CmdRightWeapMask = 0b000010000
	CmdLeftWeapMask  = 0b000100000
	CmdButtonMask    = 0b000111111

	CmdChannelMask = 0b011000000
)

const (
	//Hexbug Channel ids; sutiable for comparing with cmd and'ed with a mask: if cmd & HexbugChMask == HexbugCH2
	CH1 = 0b000000000
	CH2 = 0b001000000
	CH3 = 0b011000000 // not a mistake, go figure
	CH4 = 0b010000000
)

type StateMachine struct {
	cmdHandler func(int16)
	rcvbuf     int16
	bitcount   int
	parity     bool
}

// Hexbug creates an implementation of irrx.RxStateMachine that decodes hexbug commands.
// When a command is received, cmdHandler is called. This call comes from an interrupt
// handler so you cannot make any blocking calls and should try to keep this as quick
// as possible. It's a good idea to pass off the data to another control loop.
func NewStateMachine(cmdHandler func(int16)) *StateMachine {
	return &hexbug{
		cmdHandler: cmdHandler,
	}
}

// SetCmdHandler lets you change the callback for when a cmd is received.
func (hb *hexbug) SetCmdHandler(cmdHandler func(int16)) {
	hb.cmdHandler = cmdHandler
}

// HandleTimePair implements the irtrx.RxStateMachine interface
func (hb *hexbug) HandleTimePair(pair irtrx.TimePair) {
	on, off := pair[0], pair[1]

	if off > 1600*time.Microsecond {
		// start of frame/byte
		hb.rcvbuf = 0
		hb.bitcount = 0
		hb.parity = false
		return
	}

	// off time > 750 is a one
	if off > 750*time.Microsecond {
		hb.rcvbuf |= 1 << hb.bitcount
		// parity starts off false, if we toggle it everytime we get a one,
		// then an odd number of ones results in a "true"; voila, easy odd parity check
		hb.parity = !hb.parity
	}

	hb.bitcount++

	if hb.bitcount == 9 {
		// verify parity
		if hb.parity {
			hb.cmdHandler(hb.rcvbuf)
		}
		hb.rcvbuf = 0
		hb.bitcount = 0
		hb.parity = false
	}
}

type Cmd int16

var (
	hbStart = irtrx.TimePair{1750 * time.Microsecond, 350 * time.Microsecond}
	hbZero  = irtrx.TimePair{350 * time.Microsecond, 350 * time.Microsecond}
	hbOne   = irtrx.OnOffPair{1000 * time.Microsecond, 350 * time.Microsecond}
)

func (c Cmd) MarshalFrame() []irtrx.TimePair {
	out := [10]irtrx.TimePair{}
	out[0] = hbStart
	var parity bool = true
	for i := 1; i < 9; i++ {
		if c&1 == 1 {
			out[i] = hbOne
			parity = !parity
		} else {
			out[i] = hbZero
		}
		c >>= 1
	}

	if parity {
		out[9] = hbOne
	} else {
		out[9] = hbZero
	}

	return out[:]
}
