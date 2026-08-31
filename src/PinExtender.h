/*
__________           .___      .__  .__                 _____  .__       .__     ___ ________________    ___
\______   \ ____   __| _/____  |  | |__| ____   ____   /     \ |__| ____ |__|   /  / \__    ___/     \   \  \
 |     ___// __ \ / __ |\__  \ |  | |  |/    \ /  _ \ /  \ /  \|  |/    \|  |  /  /    |    | /  \ /  \   \  \
 |    |   \  ___// /_/ | / __ \|  |_|  |   |  (  <_> )    Y    \  |   |  \  | (  (     |    |/    Y    \   )  )
 |____|    \___  >____ |(____  /____/__|___|  /\____/\____|__  /__|___|  /__|  \  \    |____|\____|__  /  /  /
               \/     \/     \/             \/               \/        \/       \__\                 \/  /__/
                                                                                   (c) 2018-2024 alf45star
                                                                       https://github.com/alf45tar/PedalinoMini
 */

#ifndef _PIN_EXTENDER_H
#define _PIN_EXTENDER_H

#include <Arduino.h>
#include <Wire.h>

#ifndef PIN_EXTENDER_ADDRESS
#define PIN_EXTENDER_ADDRESS  0x20      // MCP23017 with A2/A1/A0 tied to GND
#endif

#if !defined(PIN_EXTENDER_SDA_PIN) || !defined(PIN_EXTENDER_SCL_PIN)
#define PIN_EXTENDER_SDA_PIN  -1        // -1 = use the board default I2C pins
#define PIN_EXTENDER_SCL_PIN  -1
#endif

// MCP23017 register map with IOCON.BANK = 0 (the power-on default), where the
// A and B register of a pair are adjacent and can be written in a single burst.
#define MCP23017_IODIR    0x00
#define MCP23017_IPOL     0x02
#define MCP23017_GPINTEN  0x04
#define MCP23017_IOCON    0x0A
#define MCP23017_GPPU     0x0C
#define MCP23017_GPIO     0x12
#define MCP23017_OLAT     0x14

//
//  MCP23017 I/O expander
//
//  Inputs are not read on demand. The whole expander is sampled once per scan
//  cycle by refresh() so that a scan costs a single I2C transaction and every
//  button sees a consistent snapshot, instead of one transaction per pedal.
//
class PinExtender {

  public:

    PinExtender(byte address = PIN_EXTENDER_ADDRESS, TwoWire *wire = &Wire) : _address(address), _wire(wire) {}

    // Starts the I2C bus, probes the expander and resets every port to a
    // pulled-up input. Called on each profile reload, like the rest of the
    // controller setup, so pedal_pin_mode() can then claim what it needs.
    bool begin()
    {
      if (!_started) {
        if (PIN_EXTENDER_SDA_PIN >= 0 && PIN_EXTENDER_SCL_PIN >= 0)
          _wire->begin(PIN_EXTENDER_SDA_PIN, PIN_EXTENDER_SCL_PIN);
        else
          _wire->begin();
        _wire->setClock(400000);
        _started = true;
      }

      _iodir = 0xFFFF;    // every port an input ...
      _gppu  = 0xFFFF;    // ... with its pull-up on
      _olat  = 0xFFFF;
      _input = 0xFFFF;    // released state of a pulled-up switch

      _wire->beginTransmission(_address);
      _found = (_wire->endTransmission() == 0);
      if (!_found) return false;

      write16(MCP23017_IPOL,    0x0000);
      write16(MCP23017_GPINTEN, 0x0000);
      write16(MCP23017_IODIR,   _iodir);
      write16(MCP23017_GPPU,    _gppu);
      write16(MCP23017_OLAT,    _olat);
      refresh();
      return true;
    }

    bool found() const { return _found; }

    // port 0..7 = GPA0..GPA7, port 8..15 = GPB0..GPB7
    void pinMode(byte port, byte mode)
    {
      const uint16_t bit = 1 << (port & 0x0F);
      switch (mode) {
        case OUTPUT:        _iodir &= ~bit; _gppu &= ~bit; break;
        case INPUT_PULLUP:  _iodir |=  bit; _gppu |=  bit; break;
        default:            _iodir |=  bit; _gppu &= ~bit; break;
      }
      if (!_found) return;
      write16(MCP23017_IODIR, _iodir);
      write16(MCP23017_GPPU,  _gppu);
    }

    void digitalWrite(byte port, byte value)
    {
      const uint16_t bit = 1 << (port & 0x0F);
      if (value == LOW) _olat &= ~bit;
      else              _olat |=  bit;
      if (_found) write16(MCP23017_OLAT, _olat);
    }

    // State sampled by the last refresh(), no I2C traffic.
    int digitalRead(byte port)
    {
      return (_input & (1 << (port & 0x0F))) ? HIGH : LOW;
    }

    // Fetches GPIOA and GPIOB in one burst (sequential addressing is on by default).
    void refresh()
    {
      if (!_found) return;
      _wire->beginTransmission(_address);
      _wire->write(MCP23017_GPIO);
      if (_wire->endTransmission() != 0) return;
      if (_wire->requestFrom((int)_address, 2) != 2) return;
      const byte a = _wire->read();
      const byte b = _wire->read();
      _input = (b << 8) | a;
    }

  private:

    void write16(byte reg, uint16_t value)
    {
      _wire->beginTransmission(_address);
      _wire->write(reg);
      _wire->write(value & 0xFF);       // register A
      _wire->write(value >> 8);         // register B
      _wire->endTransmission();
    }

    const byte  _address;
    TwoWire    *_wire;
    bool        _started = false;
    bool        _found   = false;
    uint16_t    _iodir   = 0xFFFF;
    uint16_t    _gppu    = 0xFFFF;
    uint16_t    _olat    = 0xFFFF;
    uint16_t    _input   = 0xFFFF;
};

PinExtender pinExtender;

//
//  A pinD[]/pinA[] entry is either a native GPIO number or an expander port.
//  These route it to whichever of the two it turns out to be.
//
void pedal_pin_mode(byte pin, byte mode)
{
  if (IS_PIN_EXTENDER(pin)) pinExtender.pinMode(PIN_EXTENDER_PORT(pin), mode);
  else                      ::pinMode(pin, mode);
}

int pedal_digital_read(byte pin)
{
  return IS_PIN_EXTENDER(pin) ? pinExtender.digitalRead(PIN_EXTENDER_PORT(pin)) : ::digitalRead(pin);
}

void pedal_digital_write(byte pin, byte value)
{
  if (IS_PIN_EXTENDER(pin)) pinExtender.digitalWrite(PIN_EXTENDER_PORT(pin), value);
  else                      ::digitalWrite(pin, value);
}

// True when at least one pedal is wired to the expander. A board whose pinD[]
// and pinA[] are all native GPIOs never touches the I2C bus.
bool pin_extender_used()
{
  for (byte p = 0; p < PEDALS; p++)
    if (IS_PIN_EXTENDER(PIN_D(p)) || IS_PIN_EXTENDER(PIN_A(p))) return true;
  return false;
}

// "14" for a native GPIO, "A0".."B7" for an expander port. Debug output only,
// so a small ring of buffers is enough to allow a few calls per DPRINT.
const char *pin_label(byte pin)
{
  static char label[4][8];
  static byte next = 0;

  char *s = label[next];
  next = (next + 1) % 4;
  if (IS_PIN_EXTENDER(pin))
    snprintf(s, sizeof(label[0]), "%c%d", PIN_EXTENDER_PORT(pin) < 8 ? 'A' : 'B', PIN_EXTENDER_PORT(pin) & 0x07);
  else
    snprintf(s, sizeof(label[0]), "%d", pin);
  return s;
}

//
//  ButtonConfig reading through pedal_digital_read() instead of digitalRead(),
//  so that a pedal works the same whether it sits on a GPIO or on the expander.
//
class PedalButtonConfig : public ButtonConfig {

  public:

    int readButton(uint8_t pin) override { return pedal_digital_read(pin); }
};

//
//  Same idea for Encoded4To2ButtonConfig, which cannot be subclassed for this
//  because it keeps its two pins private.
//
class PedalEncoded4To2ButtonConfig : public ButtonConfig {

  public:

    PedalEncoded4To2ButtonConfig(uint8_t pin0, uint8_t pin1, uint8_t defaultReleasedState = HIGH) :
      _pin0(pin0),
      _pin1(pin1),
      _pressedState(defaultReleasedState ^ 0x1) {}

    int readButton(uint8_t pin) override
    {
      const int s0 = pedal_digital_read(_pin0);
      const int s1 = pedal_digital_read(_pin1);

      const uint8_t virtualPin = (s0 == _pressedState) | ((s1 == _pressedState) << 1);
      return (virtualPin == pin) ? _pressedState : (_pressedState ^ 0x1);
    }

  private:

    const uint8_t _pin0;
    const uint8_t _pin1;
    const uint8_t _pressedState;
};

#endif // _PIN_EXTENDER_H
