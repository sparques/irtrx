package cheapo

// cheapo package implements an irtrx.RxStatemachine for cheap, unknown brand IR remote controls.
// I have a stack of these things; they come with LED light strips.
// The button codes are usually in order, starting from zero and increasing, left to right, top to bottom.

import (
	"time"

	"github.com/sparques/irtrx"
)

// Cheapo implements an RX statemachine for an cheap, uknown brand IR remote
type StateMachine struct {
	CmdHandler func(uint32)
	buf        uint32
	bitcount   int
}

func NewStateMachine(cmdHandler func(uint32)) *StateMachine {
	return &Cheapo{CmdHandler: cmdHandler}
}

func (c *Cheapo) HandleTimePair(pair irtrx.TimePair) {
	on, off := pair[0], pair[1]
	switch {
	case on > 7*time.Millisecond: // 9ms start of frame
		c.buf = 0
		c.bitcount = 0
		return
	case off > time.Millisecond:
		// one
		c.buf |= 1 << c.bitcount
		fallthrough
	case off < time.Millisecond:
		// zero
		c.bitcount++
	}

	if c.bitcount < 10 {
		return
	}
	c.CmdHandler(c.buf)
}
