 // Calculate Two's Complement for checksum
function two_complement(x: number) {
    return (((x & 0xFF) ^ 0xFF) + 1) & 0xFF;
}

// create DOBOT Protocol Packet
function createDobotPacket(id : number, rw : number, queued : number, payload: Buffer): Buffer {
    // Create payload part of buffer
    let payloadBuffer = pins.createBuffer(payload.length + 2);
    payloadBuffer.setNumber(NumberFormat.UInt8LE, 0, id);
    payloadBuffer.setNumber(NumberFormat.UInt8LE, 1, (rw !== 0 ? 0x01 : 0x00) | (queued !== 0 ? 0x02 : 0x00));
    payloadBuffer.write(2, payload);

    // Calculated checksum
    let sum = 0;
    for (let i = 0; i<payloadBuffer.length;i++) {
        sum += payloadBuffer.getNumber(NumberFormat.UInt8LE, i);
    }
    let checkSum = two_complement(sum & 0xFF);

    // Create final buffer to transmit
    let dobotPackageBuffer = pins.createBuffer(4 + payloadBuffer.length);
    dobotPackageBuffer[0] = 0xAA;
    dobotPackageBuffer[1] = 0xAA;
    dobotPackageBuffer.setNumber(NumberFormat.UInt8LE, 2, payload.length + 2);
    dobotPackageBuffer.write(3, payloadBuffer);
    dobotPackageBuffer.setNumber(NumberFormat.UInt8LE, dobotPackageBuffer.length - 1, checkSum);
    return dobotPackageBuffer
}

// Create X,Y,Z,R Packet
function CreatePTPPkt(cmdtype: number, x: number, y: number, z: number, r: number):Buffer {
    let buff = pins.createBuffer(17);
    buff.setNumber(NumberFormat.UInt8LE, 0, cmdtype);
    buff.setNumber(NumberFormat.Float32LE, 1, x);
    buff.setNumber(NumberFormat.Float32LE, 5, y);
    buff.setNumber(NumberFormat.Float32LE, 9, z);
    buff.setNumber(NumberFormat.Float32LE, 13, r);
    return buff;
}

pins.setPull(1, PinPullMode.PullUp);
pins.setPull(8, PinPullMode.PullUp);
serial.redirect(
    SerialPin.P1,
    SerialPin.P8,
    BaudRate.BaudRate115200
)
basic.pause(50);
basic.showIcon(IconNames.Happy);



