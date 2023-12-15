package irtrx

import (
	. "machine"
	"time"

	"github.com/sparques/pwm"
)

type TxDevice struct {
	pin    Pin
	pgroup pwm.Group
	ch     uint8
	duty   uint32
	freq   uint64
}

func NewTxDevice(pin Pin) *TxDevice {
	pin.Configure(PinConfig{Mode: PinPWM})
	pgroup := pwm.Get(pin)
	pgroup.Configure(PWMConfig{Period: uint64(1e9) / uint64(Freq38Khz)})
	ch, _ := pgroup.Channel(pin)
	pgroup.Set(ch, 0)
	return &TxDevice{
		pin:    pin,
		pgroup: pgroup,
		ch:     ch,
		duty:   pgroup.Top() / 2,
		freq:   Freq38Khz,
	}
}

func (tx *TxDevice) SendPair(pair TimePair) {
	tx.pgroup.Set(tx.ch, tx.duty)
	time.Sleep(pair[0])
	tx.pgroup.Set(tx.ch, 0)
	time.Sleep(pair[1])
}

func (tx *TxDevice) SendPairs(pairs ...TimePair) {
	for _, p := range pairs {
		tx.SendPair(p)
	}
}

func (tx *TxDevice) SendFrame(fm FrameMarshaller) {
	tx.SendPairs(fm.MarshalFrame()...)
}

func (tx *TxDevice) SendFrames(fms ...FrameMarshaller) {
	for _, fm := range fms {
		tx.SendFrame(fm)
	}
}
