//% weight=100 color=#DC22E1 block="MINTspark DOBOT" blockId="MINTspark DOBOT" icon="\uf0e7"
//% subcategories='["Basic", "Advanced", "AI"]'
//% groups='["Setup", "Move", "Work"]'
namespace mintspark_dobot {
    export enum PtpMode {
        //% block="JUMP XYZ"
        JUMP_XYZ, // JUMP mode, (x,y,z,r) is the target point in Cartesian coordinate system
        //% block="MOVJ XYZ"
        MOVJ_XYZ, // MOVJ mode, (x,y,z,r) is the target point in Cartesian coordinate system
        //% block="MOVL XYZ"
        MOVL_XYZ, //MOVL mode, (x,y,z,r) is the target point in Cartesian coordinate system
        //% block="JUMP ANGLE"
        JUMP_ANGLE, // JUMP mode, (x,y,z,r) is the target point in Joint coordinate system
        //% block="MOVJ ANGLE"
        MOVJ_ANGLE, // MOVJ mode, (x,y,z,r) is the target point in Joint coordinate system
        //% block="MOVL ANGLE"
        MOVL_ANGLE, // MOVL mode, (x,y,z,r) is the target point in Joint coordinate system
        //% block="MOVJ INC"
        MOVJ_INC, // MOVJ mode, (x,y,z,r) is the angle increment in Joint coordinate system
        //% block="MOVL INC"
        MOVL_INC, // MOVL mode, (x,y,z,r) is the Cartesian coordinate increment in Joint coordinate system
        //% block="MOVJ XYZ INC"
        MOVJ_XYZ_INC, // MOVJ mode, (x,y,z,r) is the Cartesian coordinate increment in Cartesian coordinate system
        //% block="JUMP MOVL XYZ"
        JUMP_MOVL_XYZ, // JUMP mode, (x,y,z,r) is the Cartesian coordinate increment in Cartesian coordinate system
    };

    //% weight=100
    //% group="Setup"
    //% block="Initialise DOBOT"
    //% color=#ffcc66
    export function initConnection(): void {
        pins.setPull(1, PinPullMode.PullUp);
        pins.setPull(8, PinPullMode.PullUp);
        serial.redirect(
            SerialPin.P1,
            SerialPin.P8,
            BaudRate.BaudRate115200
        )
        basic.pause(50);
        basic.showIcon(IconNames.Happy);
    }

    //% weight=60
    //% group="Move"
    //% block="Move %mode to x %x y %y z %z r %r"
    //% color=#ffcc66
    //% inlineInputMode=inline
    export function moveArm(mode: PtpMode, x: number, y: number, z: number, r: number): void {
        sendMessage(createDobotPacket(84, 1, 0, CreatePTPPkt(mode, x, y, z, r)));
    }



    // Communication functions

    function sendMessage(buffer: Buffer) : void
    {
        serial.writeBuffer(buffer);
    }

    // Calculate Two's Complement for checksum
    function two_complement(x: number) {
        return (((x & 0xFF) ^ 0xFF) + 1) & 0xFF;
    }

    // create DOBOT Protocol Packet
    function createDobotPacket(id: number, rw: number, queued: number, payload: Buffer): Buffer {
        // Create payload part of buffer
        let payloadBuffer = pins.createBuffer(payload.length + 2);
        payloadBuffer.setNumber(NumberFormat.UInt8LE, 0, id);
        payloadBuffer.setNumber(NumberFormat.UInt8LE, 1, (rw !== 0 ? 0x01 : 0x00) | (queued !== 0 ? 0x02 : 0x00));
        payloadBuffer.write(2, payload);

        // Calculated checksum
        let sum = 0;
        for (let i = 0; i < payloadBuffer.length; i++) {
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
    function CreatePTPPkt(cmdtype: number, x: number, y: number, z: number, r: number): Buffer {
        let buff = pins.createBuffer(17);
        buff.setNumber(NumberFormat.UInt8LE, 0, cmdtype);
        buff.setNumber(NumberFormat.Float32LE, 1, x);
        buff.setNumber(NumberFormat.Float32LE, 5, y);
        buff.setNumber(NumberFormat.Float32LE, 9, z);
        buff.setNumber(NumberFormat.Float32LE, 13, r);
        return buff;
    }
}