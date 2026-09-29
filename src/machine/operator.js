// The person standing at the machine. Some things only a person does: open
// and shut the enclosure door, put stock on the plate. A job file can ask for
// such an action with a firmware message, `M118 @<action>`, which the
// firmware prints when it reaches that line (not when the PC sends it), and
// the operator carries it out on the physical machine:
//
//   M118 @door open 4     open the door, shut it again after 4 s
//   M118 @door open       open the door (and leave it open)
//   M118 @door close      shut the door
//
// The operator is on the PC side of the USB cable: it reads the serial port
// like any other program there, and acts only through physical parts.

export class Operator {
  constructor(machine) {
    this.m = machine;
    this.actions = [];                 // [t, text] for the log
    machine.host.listeners.push((msg, t) => { if (msg.startsWith('@')) this.act(msg.slice(1), t); });
  }
  act(text, t) {
    const [what, verb, arg] = text.trim().split(/\s+/);
    const sim = this.m.sim;
    this.actions.push([t, text]);
    if (what === 'door' && this.m.enclosure) {
      const enc = this.m.enclosure;
      if (verb === 'open') {
        enc.setDoor(true);
        const hold = parseFloat(arg);
        if (hold > 0) sim.at(t + hold, () => enc.setDoor(false), 'operator');
      } else if (verb === 'close') enc.setDoor(false);
    }
  }
}
