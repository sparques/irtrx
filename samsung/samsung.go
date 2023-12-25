// samsung implements an irtrx.RxStateMachine that can decode Samsung IR signals.
package samsung

import (
	"errors"
	"time"

	"github.com/sparques/irtrx"
)

type StateMachine struct {
	CmdHandler func(Frame)

	buf      uint32
	bitcount int
}

var (
	// ErrFrameAlloc is returned when an attempt to unmarshal to a nil Frame is done--the frame must be allocated ahead of time
	ErrFrameAlloc = errors.New("tried to unmarshal to unallocated frame")
)

type Frame struct {
	Addr uint16
	Cmd  uint16
}

func NewStateMachine(cmdHandler func(Frame)) *StateMachine {
	return &StateMachine{CmdHandler: cmdHandler}
}

func (sm *StateMachine) HandleTimePair(pair irtrx.TimePair) {
	on, off := pair[0], pair[1]
	switch {
	case on > 200*time.Millisecond:
		return
	case off > 3*time.Millisecond:
		//start of frame
		if on > 3*time.Millisecond {
			sm.buf = 0
			sm.bitcount = 0
		}
		return
	}

	if on > time.Millisecond {
		sm.buf |= 1 << sm.bitcount
	}
	sm.bitcount++

	if sm.bitcount != 32 {
		return
	}

	var f Frame
	f.UnmarshalFrame(sm.buf)
	sm.CmdHandler(f)
}

var (
	StartPair = irtrx.TimePair{6 * time.Millisecond, 3 * time.Millisecond}
	ZeroPair  = irtrx.TimePair{562 * time.Microsecond, 1687 * time.Microsecond}
	OnePair   = irtrx.TimePair{562 * time.Microsecond, 562 * time.Microsecond}
)

func (f *Frame) MarshalFrame() []irtrx.TimePair {
	out := make([]irtrx.TimePair, 34)

	// start of frame
	out[0] = StartPair

	buf := uint32(f.Cmd)<<16 | uint32(f.Addr)

	for bit := 0; bit < 32; bit++ {
		if (buf>>bit)&1 == 1 {
			out[bit+1] = OnePair
		} else {
			out[bit+1] = ZeroPair
		}
	}

	// Stop Bit is a Zero
	out[33] = ZeroPair

	return out
}

func (f *Frame) UnmarshalFrame(buf uint32) error {
	if f == nil {
		return ErrFrameAlloc
	}
	f.Addr = uint16(buf & 0xFFFF)
	f.Cmd = uint16((buf >> 16) & 0xFFFF)
	return nil
}
