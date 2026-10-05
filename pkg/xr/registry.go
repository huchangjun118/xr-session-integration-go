package xr

import (
	"fmt"
	"sync"
)

// registry tracks normalized devices and their stable identities, bindings
// and hot-plug presence.
//
// Identity rules:
//
//   - one logical device per (kind, handedness); a headset is handedness
//     none, controllers/hands/eyes have left/right members;
//   - the normalized DeviceID never changes across unplug/replug;
//   - Bindings (application configuration) outlive a physical absence;
//   - registration order is the canonical snapshot ordering.
type registry struct {
	mu      sync.RWMutex
	order   []DeviceID
	byID    map[DeviceID]*Device
	keyToID map[string]DeviceID
}

func newRegistry() *registry {
	return &registry{byID: make(map[DeviceID]*Device), keyToID: make(map[string]DeviceID)}
}

// canonicalID derives the stable identity for a logical device.
func canonicalID(kind Kind, hand Handedness) DeviceID {
	switch kind {
	case KindHeadset:
		return DeviceID("headset/0")
	case KindEyes:
		return DeviceID("eyes/0")
	case KindController:
		return DeviceID(fmt.Sprintf("controller/%s", hand))
	case KindHand:
		return DeviceID(fmt.Sprintf("hand/%s", hand))
	default:
		return DeviceID(fmt.Sprintf("kind%d/%s", kind, hand))
	}
}

// ensure returns the normalized device for a source-specific key, creating
// the stable identity on first sight. The source key is recorded but does not
// define identity, so a re-plugged device keeps the same ID and bindings.
func (r *registry) ensure(key string, kind Kind, hand Handedness, source string) Device {
	r.mu.Lock()
	defer r.mu.Unlock()
	id := canonicalID(kind, hand)
	if key != "" {
		if mapped, ok := r.keyToID[key]; ok {
			id = mapped
		} else {
			r.keyToID[key] = id
		}
	}
	d, ok := r.byID[id]
	if !ok {
		d = &Device{
			ID:         id,
			Kind:       kind,
			Hand:       hand,
			SourceName: source,
			Bindings:   make(map[string]string),
		}
		r.byID[id] = d
		r.order = append(r.order, id)
	} else if source != "" {
		d.SourceName = source
	}
	return *d
}

// lookupKey returns the normalized device previously registered for a source
// key.
func (r *registry) lookupKey(key string) (Device, bool) {
	r.mu.RLock()
	defer r.mu.RUnlock()
	id, ok := r.keyToID[key]
	if !ok {
		return Device{}, false
	}
	d, ok := r.byID[id]
	if !ok {
		return Device{}, false
	}
	return *d, true
}

// registerID explicitly maps a source key to an existing/new canonical
// device and marks it online.
func (r *registry) registerID(key string, kind Kind, hand Handedness, source string) Device {
	d := r.ensure(key, kind, hand, source)
	return r.attach(d.ID)
}

func (r *registry) attach(id DeviceID) Device {
	r.mu.Lock()
	defer r.mu.Unlock()
	d, ok := r.byID[id]
	if !ok {
		return Device{}
	}
	d.Online = true
	return *d
}

func (r *registry) detach(id DeviceID) Device {
	r.mu.Lock()
	defer r.mu.Unlock()
	d, ok := r.byID[id]
	if !ok {
		return Device{}
	}
	d.Online = false
	return *d
}

func (r *registry) device(id DeviceID) (Device, bool) {
	r.mu.RLock()
	defer r.mu.RUnlock()
	d, ok := r.byID[id]
	if !ok {
		return Device{}, false
	}
	return *d, true
}

// list returns devices in canonical registration order.
func (r *registry) list() []Device {
	r.mu.RLock()
	defer r.mu.RUnlock()
	out := make([]Device, 0, len(r.order))
	for _, id := range r.order {
		out = append(out, *r.byID[id])
	}
	return out
}

// setBinding stores a stable per-device configuration entry that survives
// unplug/replug.
func (r *registry) setBinding(id DeviceID, key, value string) {
	r.mu.Lock()
	defer r.mu.Unlock()
	if d, ok := r.byID[id]; ok {
		d.Bindings[key] = value
	}
}

func (r *registry) binding(id DeviceID, key string) (string, bool) {
	r.mu.RLock()
	defer r.mu.RUnlock()
	if d, ok := r.byID[id]; ok {
		v, ok := d.Bindings[key]
		return v, ok
	}
	return "", false
}
