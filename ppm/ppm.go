package ppm

/*
ppm.go - PPM for IR

Might work exactly the same as though connected to a regular PPM radio receiver.

Note, with a carrier of 38kHz, the most graduations you get per channel is 38, slightly
more than 5 bits worth. This is because ppm uses a 1ms to 2ms pulse per channel and
1ms * 38kHz = 38.

So, fairly limited, but still pretty good.

If using to drive a 180 degree servo, that means each graduation is 4.7 degrees (180/38).


## Example

    const irPin = machine.GPIOX
    ppm := irrx.PPM()
    rx := irrx.NewRxDevice(irPin, ppm)
    rx.Start()

    for {
        X := float32(ppm.Channel(0)-1500)/500
        // X ranges from -1 to 1
        servo.Set(X)
        time.Sleep(100 * time.Millisecond)
    }

*/

import (
	"time"

	"github.com/sparques/irtrx"
)

const minimumTimeBetweenFrames = 6 * time.Millisecond

var (
	SafeChannelsMid    = [16]time.Duration{1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond, 1500 * time.Microsecond}
	SafeChannelsBottom = [16]time.Duration{1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond, 1000 * time.Microsecond}
)

type StateMachine struct {
	// where we store the values we've decoded
	channels [16]time.Duration
	// safeChannels are what we set channels to if we exceed Timout
	safeChannels [16]time.Duration

	// if we haven't received a frame in Timeout amount of time, we return
	// values from safeChannels
	Timeout time.Duration

	currentCh int
	last      time.Time
}

func NewStateMachine() *StateMachine {
	def := &PPM{
		Timeout:      100 * minimumTimeBetweenFrames,
		safeChannels: SafeChannelsMid,
	}
	return def
}

func (p *PPM) HandleTimePair(pair irtrx.TimePair) {
	on, off := pair[0], pair[1]
	if on > ppmMinimumTimeBetweenFrames {
		p.last = time.Now()
		p.currentCh = 0
		return
	}

	// prevent out-of-spec signals from panicking us.
	if p.currentCh >= len(p.channels) {
		return
	}

	p.channels[p.currentCh] = on + off
	p.currentCh++
}

func (p *PPM) SetSafeChannels(sc [16]time.Duration) {
	p.safeChannels = sc
}

func (p *PPM) IsSafe() bool {
	return time.Since(p.last) > p.Timeout
}

// Channel returns the duration of the the pulse for the given channel.
// Converting the time.Duration value into something more useful is left
// to the caller.
// If ppm.Timeout has been exceeded, the safe value for the channel is
// returned.
func (p *PPM) Channel(ch int) time.Duration {
	if time.Since(p.last) > p.Timeout {
		return p.safeChannels[ch]
	}
	return p.channels[ch]
}

// Channels returns all the channels.
// If ppm.Timeout has been exceeded, the safe values are returned
func (p *PPM) Channels() [16]time.Duration {
	if time.Since(p.last) > p.Timeout {
		return p.safeChannels
	}
	return p.channels
}

func DurationToFloat32(d time.Duration) float32 {
	return float32(2*d-3*time.Millisecond) / float32(time.Millisecond)
}

/*
type PPMCalibrator struct {
	PPM *PPM
	max [16]time.Duration
	min [16]time.Duration
}

func NewPPMCalibrator() *PPMCalibrator {
	pc := &PPMCalibrator{PPM: PPM()}
	for ch := range pc.max {
		pc.max[ch] = time.Millisecond
		pc.min[ch] = 2 * time.Millisecond
	}
	return pc
}

func (pc *PPMCalibrator) Update(chs [16]time.Duration) {
	for ch := range chs {
		if pc.max[ch] < chs[ch] {
			pc.max[ch] = chs[ch]
			continue
		}
		if pc.min[ch] > chs[ch] {
			pc.min[ch] = chs[ch]
		}
	}
}

func (pc *PPMCalibrator) HandleOnOff(on, off time.Duration) {
	pc.PPM.HandleOnOff(on, off)
	if pc.PPM.currentCh == 0 {
		pc.Update(pc.PPM.channels)
	}
}

func (pc *PPMCalibrator) Channel(ch int) float32 {
	return float32(2*pc.PPM.Channel(ch)-(pc.max[ch]+pc.min[ch])) / float32(pc.max[ch]-pc.min[ch])
}
*/
