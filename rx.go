package irtrx

import (
	. "machine"
	"time"
)

type RxDevice struct {
	pin          Pin
	pulseCount   int
	lastPulse    time.Time
	lastHigh     time.Duration
	stateMachine RxStateMachine
}

type RxStateMachine interface {
	HandleTimePair(TimePair)
}

type multiRxStateMachine []RxStateMachine

func (mrsm multiRxStateMachine) HandleTimePair(pair TimePair) {
	for i := range mrsm {
		mrsm[i].HandleTimePair(pair)
	}
}

// MultiRxStateMachine accepts a list of RxStateMachines and returns an object
// that also implements RxStateMachine. When HandleTimePair is called against it,
// it calls HandleTimePair against all the RxStateMachines used to define it.
// In this way, you an effectively multiplex multiple RxStateMachines under
// a single IR receiver.
// E.G.:
//
//	mult := irtrx.MultiRxStateMachine(hexbug.NewStateMachine(hbHandler),nec.NewStateMachine(necHandler))
//	rxd := irtrx.NewRxDevice(pin, mult)
func MultiRxStateMachine(rsm ...RxStateMachine) multiRxStateMachine {
	return multiRxStateMachine(rsm)
}

func NewRxDevice(pin Pin, rsm RxStateMachine) *RxDevice {
	// the most common receivers have a pull up pin builtin
	// but in the future, may want to add the option to use PinPullupInput
	pin.Configure(PinConfig{Mode: PinInput})
	return &RxDevice{
		pin:          pin,
		stateMachine: rsm,
	}
}

func (rx *RxDevice) interruptHandler(interruptPin Pin) {
	ptime := time.Now()
	if interruptPin.Get() {
		// pin high
		rx.stateMachine.HandleTimePair(TimePair{rx.lastHigh, time.Since(rx.lastPulse)})
	} else {
		rx.lastHigh = time.Since(rx.lastPulse)
	}
	rx.lastPulse = ptime
}

func (rx *RxDevice) invertedInterruptHandler(interruptPin Pin) {
	ptime := time.Now()
	if interruptPin.Get() {
		// pin high
		rx.lastHigh = time.Since(rx.lastPulse)
	} else {
		rx.stateMachine.HandleTimePair(TimePair{rx.lastHigh, time.Since(rx.lastPulse)})
	}
	rx.lastPulse = ptime
}

// Start sets the interrupt handler and thus starts processing signals.
// Use Start() if your RxStateMachine uses on-off pairs, e.g. Hexbug or PPM.
func (rx *RxDevice) Start() {
	rx.pin.SetInterrupt(PinFalling|PinRising, rx.interruptHandler)
}

// StartInverted sets the interrupt handler and thus starts processing signals.
// Use StartInverted if your RxStatemachine uses off-on pairs, e.g. NEC.
func (rx *RxDevice) StartInverted() {
	rx.pin.SetInterrupt(PinFalling|PinRising, rx.invertedInterruptHandler)
}

// Stop disables the interrupt handler.
func (rx *RxDevice) Stop() {
	rx.pin.SetInterrupt(PinFalling|PinRising, nil)
}
