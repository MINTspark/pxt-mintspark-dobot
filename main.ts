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

    export enum JogCommand {
        IDEL, //Void
        AP_DOWN, // X+/Joint1+
        AN_DOWN, // X-/Joint1-
        BP_DOWN, // Y+/Joint2+
        BN_DOWN, // Y-/Joint2-
        CP_DOWN, // Z+/Joint3+
        CN_DOWN, // Z-/Joint3-
        DP_DOWN, // R+/Joint4+
        DN_DOWN // R-/Joint4-
    };

    export enum CoordinateSystem {
        //% block="Cartesian"
        Cartesian = 0,
        //% block="Joint"
        Joint = 1
    }

    export enum PtpMoveMode {
        //% block="Jump"
        Jump = 0,
        //% block="Joint"
        Joint = 1,
        //% block="Linear"
        Linear = 2  
    }

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

    //% weight=95
    //% group="Setup"
    //% block="Clear Alarm"
    //% color=#ffcc66
    export function clearAlarm(): void {
        sendMessage(createDobotPacket(20, 1, 0, pins.createBuffer(0)));
    }

    //% weight=90
    //% group="Setup"
    //% block="Start Homing"
    //% color=#ffcc66
    export function startHome(): void {
        let buff = pins.createBuffer(4);
        buff.fill(0);
        sendMessage(createDobotPacket(31, 1, 0, buff));
    }

    //% weight=80
    //% group="Setup"
    //% block="Set Jog %mode speed %speed accel %acceleration"
    //% color=#ffcc66
    export function setJogSpeedAndAcceleration(system: CoordinateSystem, speed: number, acceleration: number): void {
        let cmd = system == CoordinateSystem.Joint ? 70 : 71;
        let buff = pins.createBuffer(32);
        bufferSetFloatArray(buff, 0, [speed, speed, speed, speed, acceleration, acceleration, acceleration, acceleration]);
        sendMessage(createDobotPacket(cmd, 1, 0, buff));
    }

    //% weight=75
    //% group="Setup"
    //% block="Set PTP Joint speed %speed accel %acceleration"
    //% color=#ffcc66
    export function setPtpJointSpeedAndAcceleration(speed: number, acceleration: number): void {
        let buff = pins.createBuffer(32);
        bufferSetFloatArray(buff, 0, [speed, speed, speed, speed, acceleration, acceleration, acceleration, acceleration]);
        sendMessage(createDobotPacket(80, 1, 0, buff));
    }

    //% weight=74
    //% group="Setup"
    //% block="Set PTP Cartesian speed %speed accel %acceleration"
    //% color=#ffcc66
    export function setPtpCartesianSpeedAndAcceleration(linearSpeed: number, effectorSpeed: number, linearAcceleration: number, effectorAcceleration: number): void {
        let buff = pins.createBuffer(16);
        bufferSetFloatArray(buff, 0, [linearSpeed, effectorSpeed, linearAcceleration, effectorAcceleration]);
        sendMessage(createDobotPacket(81, 1, 0, buff));
    }

    //% weight=70
    //% group="Setup"
    //% block="Set PTP Jump height %height z-limit %zLimit"
    //% color=#ffcc66
    export function setPtpJumpParameters(height: number, zLimit: number): void {
        let buff = pins.createBuffer(8);
        bufferSetFloatArray(buff, 0, [height, zLimit]);
        sendMessage(createDobotPacket(82, 1, 0, buff));
    }

    //% weight=50
    //% group="Move"
    //% block="Jog %system command %jogComand"
    //% color=#ffcc66
    //% inlineInputMode=inline
    export function moveJog(system: CoordinateSystem, jogComand: JogCommand): void {
        let buff = pins.createBuffer(2);
        buff.setNumber(NumberFormat.UInt8LE, 0, system)
        buff.setNumber(NumberFormat.UInt8LE, 1, jogComand)
        sendMessage(createDobotPacket(73, 1, 0, buff));
    }

    //% weight=40
    //% group="Move"
    //% block="Move Cartesian mode %mode to x %x y %y z %z r %r"
    //% color=#ffcc66
    //% inlineInputMode=inline
    export function movePtpCartesian(mode: PtpMoveMode, x: number, y: number, z: number, r: number): void {
        let ptpMode: PtpMode;
        switch (mode) {
            case PtpMoveMode.Jump:
                ptpMode = PtpMode.JUMP_XYZ;
                break;
            case PtpMoveMode.Joint:
                ptpMode = PtpMode.MOVJ_XYZ;
                break;
            default:
                ptpMode = PtpMode.MOVL_XYZ;
        }

        sendMessage(createDobotPacket(84, 1, 0, CreatePtpPayload(ptpMode, x, y, z, r)));
    }

    //% weight=38
    //% group="Move"
    //% block="Move Joint mode %mode to J1 %j1 J2 %j2 J3 %j3 J4 %j4"
    //% color=#ffcc66
    //% inlineInputMode=inline
    export function movePtpJoint(mode: PtpMoveMode, j1: number, j2: number, j3: number, j4: number): void {
        let ptpMode: PtpMode;
        switch (mode) {
            case PtpMoveMode.Jump:
                ptpMode = PtpMode.JUMP_ANGLE;
                break;
            case PtpMoveMode.Joint:
                ptpMode = PtpMode.MOVJ_ANGLE;
                break;
            default:
                ptpMode = PtpMode.MOVL_ANGLE;
        }

        sendMessage(createDobotPacket(84, 1, 0, CreatePtpPayload(ptpMode, j1, j2, j3, j4)));
    }

    //% weight=36
    //% group="Move"
    //% block="Move Cartesian Increment x %x y %y z %z r %r"
    //% color=#ffcc66
    //% inlineInputMode=inline
    export function movePtpCartesianIncrement(x: number, y: number, z: number, r: number): void {
        sendMessage(createDobotPacket(84, 1, 0, CreatePtpPayload(PtpMode.MOVJ_XYZ_INC, x, y, z, r)));
    }

    //% weight=34
    //% group="Move"
    //% block="Move Joint Increment J1 %j1 J2 %j2 J3 %j3 J4 %j4"
    //% color=#ffcc66
    //% inlineInputMode=inline
    export function movePtpJointIncrement(j1: number, j2: number, j3: number, j4: number): void {
        sendMessage(createDobotPacket(84, 1, 0, CreatePtpPayload(PtpMode.MOVJ_INC, j1, j2, j3, j4)));
    }

    //% weight=70
    //% group="Move"
    //% block="Stop Immediate"
    //% color=#ffcc66
    export function stopCommand():void{
        sendMessage(createDobotPacket(242, 1, 0, pins.createBuffer(0)));
    }

    export enum GripperState{
        Open = 0,
        Closed = 1
    }

    export enum PumpState {
        Off = 0,
        On = 1
    }


    //% weight=30
    //% group="Effector"
    //% block="Set pump %pumpState suction cup %gripperState"
    //% color=#ffcc66
    export function setSuctionCup(pumpState: PumpState, suckerState: GripperState): void {
        let buff = pins.createBuffer(2);
        buff.setNumber(NumberFormat.UInt8LE, 0, pumpState)
        buff.setNumber(NumberFormat.UInt8LE, 1, suckerState)
        sendMessage(createDobotPacket(62, 1, 0, buff));
    }

    //% weight=29
    //% group="Effector"
    //% block="Set pump %pumpState Gripper %gripperState"
    //% color=#ffcc66
    export function setGripper(pumpState: PumpState, gripperState: GripperState): void {
        let buff = pins.createBuffer(2);
        buff.setNumber(NumberFormat.UInt8LE, 0, pumpState)
        buff.setNumber(NumberFormat.UInt8LE, 1, gripperState)
        sendMessage(createDobotPacket(63, 1, 0, buff));
    }

    // Communication functions

    // Send message over serial
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

    // Create point to point X,Y,Z,R Payload
    function CreatePtpPayload(cmdtype: number, x: number, y: number, z: number, r: number): Buffer {
        let buff = pins.createBuffer(17);
        buff.setNumber(NumberFormat.UInt8LE, 0, cmdtype);
        buff.setNumber(NumberFormat.Float32LE, 1, x);
        buff.setNumber(NumberFormat.Float32LE, 5, y);
        buff.setNumber(NumberFormat.Float32LE, 9, z);
        buff.setNumber(NumberFormat.Float32LE, 13, r);
        return buff;
    }

    // Set array of numbers as 4 byte little endian foat values in buffer
    function bufferSetFloatArray(buffer: Buffer, offset: number, numbers: number[]) {
        let counter = 0;
        for (let n of numbers) {
            buffer.setNumber(NumberFormat.Float32LE, offset + counter * 4, n);
            counter++;
        }
    }
}