//% weight=100 color=#DC22E1 block="MINTspark DOBOT" blockId="MINTspark DOBOT" icon="\uf0e7"
//% subcategories='["Basic", "Advanced", "AI"]'
//% groups='["Setup", "Move", "Remote"]'
namespace mintspark_dobot {


    // TODO:
    // Add protection for joint jogging
    // Have separate functions for jogging cartesian and joint
    // Set joint boundaries for safety, add cartesian R soft boundary
    // Add Alert message checking

  
    class CartesianPosition{
        X: number;
        Y: number;
        Z: number;
        R: number;

        constructor(x: number, y: number, z: number, r: number)
        {
            this.X=x;
            this.Y = y;
            this.Z = z;
            this.R = r;
        }

        toString():string{
            return "x:" + this.X + " y:" + this.Y + " z:" + this.Z + " r:" + this.R;
        }
    }

    class JointPosition {
        J1: number;
        J2: number;
        J3: number;
        J4: number;

        constructor(j1: number, j2: number, j3: number, j4: number) {
            this.J1 = j1;
            this.J2 = j2;
            this.J3 = j3;
            this.J4 = j4;
        }

        toString(): string {
            return "J1:" + this.J1 + " J2:" + this.J2 + " J3:" + this.J3 + " J4:" + this.J4;
        }
    }
    
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
        IDEL = 0, //Void
        AP_DOWN = 1, // X+/Joint1+
        AN_DOWN = 2, // X-/Joint1-
        BP_DOWN = 3, // Y+/Joint2+
        BN_DOWN = 4, // Y-/Joint2-
        CP_DOWN = 5, // Z+/Joint3+
        CN_DOWN = 6, // Z-/Joint3-
        DP_DOWN = 7, // R+/Joint4+
        DN_DOWN = 8 // R-/Joint4-
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

    export enum GripperState {
        //% block="Open"
        Open = 0,
        //% block="Closed"
        Closed = 1
    }

    export enum SuctionCupState {
        //% block="Blow"
        Blow = 0,
        //% block="Suck"
        Suck = 1
    }

    let fixedCartesianPositions: CartesianPosition[] = [];
    let isInitialised = false;
    let startPosition: CartesianPosition = new CartesianPosition(210, 0, 80, 10);
    let currentPositionCartesian: CartesianPosition;
    let currentPositionJoint: JointPosition;

    let zMinusLimit = -43;
    let zPlusLimit = 35;
    let xMinusLimit = 160;
    let xPlusLimit = 240;
    let yMinusLimit = -180;
    let yPlusLimit = 180;
    let rMinusLimit = -130;
    let rPlusLimit = 130;
    let j1MinusLimit = 0;
    let j1PlusLimit = 0;
    let j2MinusLimit = 0;
    let j2PlusLimit = 0;
    let j3MinusLimit = 0;
    let j3PlusLimit = 0;
    let j4MinusLimit = 0;
    let j4PlusLimit = 0;

    //% weight=140
    //% group="Setup"
    //% block="Initialise DOBOT"
    //% color=#1e90ff
    export function initialiseDobot(): void {
        requestPosition();
    }

    //% weight=110
    //% group="Setup"
    //% block="Set Startposition to|x %x y %y z %z r %r"
    //% color=#1e90ff
    //% inlineInputMode=external
    export function setStartPosition(x: number, y: number, z: number, r: number): void {
        startPosition = new CartesianPosition(x, y, z, r);
        movePtpJoint(PtpMoveMode.Joint, 0, 20, 30, 0);
    } 

    //% weight=105
    ////% subcategory="Advanced"
    //% group="Setup"
    //% block="Set cartesian soft limits|X+: %xPlus X-: %xMinus Y+: %yPlus Y-: %yMinus Z+: %zPlus Z-: %zMinus R+: %rPlus R-: %rMinus"
    //% color=#1e90ff
    //% inlineInputMode=external
    export function setCartesianSoftLimits(xPlus: number, xMinus: number, yPlus: number, yMinus: number, zPlus: number, zMinus: number, rPlus: number, rMinus: number, ): void {
        xPlusLimit = xPlus;
        xMinusLimit = xMinus;
        yPlusLimit = yPlus;
        yMinusLimit = yMinus;
        zPlusLimit = zPlus;
        zMinusLimit = zMinus;
        rPlusLimit = rPlus;
        rMinusLimit = rMinus;
    }

    //% weight=103
    ////% subcategory="Advanced"
    //% group="Setup"
    //% block="Set fixed Position %index|x %x y %y z %z r %r"
    //% color=#1e90ff
    //% inlineInputMode=external
    export function setFixedPosition(index: number, x: number, y: number, z: number, r: number): void {
        fixedCartesianPositions[index] = new CartesianPosition(x, y, z, r);
    }


    //% weight=120
    //% group="Setup"
    //% block="Current Cartesian Position"
    //% color=#1e90ff
    export function getCurrentPositionCartesian(): string {
        if (currentPositionCartesian != null)
        {
            return currentPositionCartesian.toString();
        }
        return "Unknown";
    }

    //% weight=118
    //% group="Setup"
    //% block="Current Joint Position"
    //% color=#1e90ff
    export function getCurrentPositionJoint(): string {
        if (currentPositionJoint != null) {
            return currentPositionJoint.toString();
        }
        return "Unknown";
    }

    //% weight=95
    //% subcategory="Advanced"
    //% group="Setup"
    //% block="Clear Alarm"
    //% color=#1e90ff
    export function clearAlarm(): void {
        sendMessage(createDobotPacket(21, 1, 0, pins.createBuffer(0)));
    }

    //% weight=90
    //% subcategory="Advanced"
    //% group="Setup"
    //% block="Start Homing"
    //% color=#1e90ff
    export function startHome(): void {
        let buff = pins.createBuffer(4);
        buff.fill(0);
        sendMessage(createDobotPacket(31, 1, 0, buff));
    }

    //% weight=80
    //% group="Setup"
    //% block="Set Jog %mode speed %speed accel %acceleration"
    //% color=#1e90ff
    //% inlineInputMode=inline
    export function setJogSpeedAndAcceleration(system: CoordinateSystem, speed: number, acceleration: number): void {
        let cmd = system == CoordinateSystem.Joint ? 70 : 71;
        let buff = pins.createBuffer(32);
        bufferSetFloatArray(buff, 0, [speed, speed, speed, speed, acceleration, acceleration, acceleration, acceleration]);
        sendMessage(createDobotPacket(cmd, 1, 0, buff));
    }

    ////% weight=75
    ////% subcategory="Advanced"
    ////% group="Setup"
    ////% block="Set joint speed %speed accel %acceleration"
    ////% color=#1e90ff
    ////% inlineInputMode=inline
    //export function setPtpJointSpeedAndAcceleration(speed: number, acceleration: number): void {
    //    let buff = pins.createBuffer(32);
    //    bufferSetFloatArray(buff, 0, [speed, speed, speed, speed, acceleration, acceleration, acceleration, acceleration]);
    //    sendMessage(createDobotPacket(80, 1, 0, buff));
    //}

    ////% weight=74
    ////% subcategory="Advanced"
    ////% group="Setup"
    ////% block="Set linear speed %linearSpeed accel %linearAcceleration effector speed %effectorSpeed accel %effectorAcceleration"
    ////% color=#1e90ff
    //export function setPtpLinearSpeedAndAcceleration(linearSpeed: number, linearAcceleration: number, effectorSpeed: number, effectorAcceleration: number): void {
    //    let buff = pins.createBuffer(16);
    //    bufferSetFloatArray(buff, 0, [linearSpeed, effectorSpeed, linearAcceleration, effectorAcceleration]);
    //    sendMessage(createDobotPacket(81, 1, 0, buff));
    //}

    //% weight=72
    //% group="Setup"
    //% block="Set PTP speed %speed accel %acceleration"
    //% color=#1e90ff
    //% inlineInputMode=inline
    export function setPtpSpeedAndAcceleration(speed: number, acceleration: number): void {
        let buff = pins.createBuffer(8);
        bufferSetFloatArray(buff, 0, [speed, acceleration]);
        sendMessage(createDobotPacket(83, 1, 0, buff));
    }

    //% weight=70
    //% subcategory="Advanced"
    //% group="Setup"
    //% block="Set PTP Jump height %height z-limit %zLimit"
    //% color=#1e90ff
    //% inlineInputMode=inline
    export function setPtpJumpParameters(height: number, zLimit: number): void {
        let buff = pins.createBuffer(8);
        bufferSetFloatArray(buff, 0, [height, zLimit]);
        sendMessage(createDobotPacket(82, 1, 0, buff));
    }

    //% weight=50
    //% subcategory="Advanced"
    //% group="Move"
    //% block="Jog %system command %jogComand"
    //% color=#1e90ff
    //% inlineInputMode=inline
    export function moveJog(system: CoordinateSystem, jogComand: JogCommand): void {
        if (system == CoordinateSystem.Cartesian && ProtectCartesianJogCommand(jogComand))
        {   
            jogComand = JogCommand.IDEL;
            music.play(music.tonePlayable(Note.C, music.beat(BeatFraction.Eighth)), music.PlaybackMode.InBackground)
        }

        let buff = pins.createBuffer(2);
        buff.setNumber(NumberFormat.UInt8LE, 0, system)
        buff.setNumber(NumberFormat.UInt8LE, 1, jogComand)
        sendMessage(createDobotPacket(73, 1, 0, buff));
    }

    //% weight=40
    //% group="Move"
    //% block="Move Cartesian mode %mode to x %x y %y z %z r %r"
    //% color=#1e90ff
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

        sendMessage(createDobotPacket(84, 1, 1, CreatePtpPayload(ptpMode, x, y, z, r)));
    }

    //% weight=42
    //% group="Move"
    //% block="Move to Startposition mode %mode"
    //% color=#1e90ff
    export function moveToStartPosition(mode: PtpMoveMode): void {
        movePtpCartesian(PtpMoveMode.Joint, startPosition.X, startPosition.Y, startPosition.Z, startPosition.R);
    }

    //% weight=41
    //% subcategory="Advanced"
    //% group="Move"
    //% block="Move to Postion %positionIndex mode %mode"
    //% color=#1e90ff
    export function moveToFixedPosition(mode: PtpMoveMode, positionIndex: number): void {
        let position = fixedCartesianPositions[positionIndex];
        if (position != null)
        {
            movePtpCartesian(PtpMoveMode.Joint, position.X, position.Y, position.Z, position.R);
        }
    }

    //% weight=38
    //% group="Move"
    //% block="Move Joint mode %mode to J1 %j1 J2 %j2 J3 %j3 J4 %j4"
    //% color=#1e90ff
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

        sendMessage(createDobotPacket(84, 1, 1, CreatePtpPayload(ptpMode, j1, j2, j3, j4)));
    }

    //% weight=36
    //% group="Move"
    //% block="Move Cartesian Increment x %x y %y z %z r %r"
    //% color=#1e90ff
    //% inlineInputMode=inline
    export function movePtpCartesianIncrement(x: number, y: number, z: number, r: number): void {
        sendMessage(createDobotPacket(84, 1, 0, CreatePtpPayload(PtpMode.MOVJ_XYZ_INC, x, y, z, r)));
    }

    //% weight=34
    //% group="Move"
    //% block="Move Joint Increment J1 %j1 J2 %j2 J3 %j3 J4 %j4"
    //% color=#1e90ff
    //% inlineInputMode=inline
    export function movePtpJointIncrement(j1: number, j2: number, j3: number, j4: number): void {
        sendMessage(createDobotPacket(84, 1, 0, CreatePtpPayload(PtpMode.MOVJ_INC, j1, j2, j3, j4)));
    }

    //% weight=70
    //% group="Move"
    //% block="Stop Immediate"
    //% color=#1e90ff
    export function stopCommand():void{
        sendMessage(createDobotPacket(241, 1, 0, pins.createBuffer(0)));
    }

    //% weight=30
    //% group="Effector"
    //% block="Set suction cup %suckerState"
    //% color=#1e90ff
    export function setSuctionCup(suckerState: SuctionCupState): void {
        let gripperState = suckerState == SuctionCupState.Suck ? GripperState.Closed : GripperState.Open;
        setGripper(gripperState);
    }

    //% weight=25
    //% group="Effector"
    //% block="Set pump OFF"
    //% color=#1e90ff
    export function setPumpOff(): void {
        let buff = pins.createBuffer(2);
        buff.setNumber(NumberFormat.UInt8LE, 0, 0)
        buff.setNumber(NumberFormat.UInt8LE, 1, 0)
        sendMessage(createDobotPacket(63, 1, 1, buff));
    }

    //% weight=29
    //% group="Effector"
    //% block="Set Gripper %gripperState"
    //% color=#1e90ff
    export function setGripper(gripperState: GripperState): void {
        let buff = pins.createBuffer(2);
        buff.setNumber(NumberFormat.UInt8LE, 0, 1)
        buff.setNumber(NumberFormat.UInt8LE, 1, gripperState)
        sendMessage(createDobotPacket(63, 1, 1, buff));
    }

    let remoteControlActive = false;

    //% weight=10
    //% subcategory="Advanced"
    //% group="Remote"
    //% block="Set Remote Channel to %channel"
    //% color=#1e90ff
    export function setRemoteChannel(channel: number): void {    
        radio.setGroup(channel);
    }

    //% weight=9
    //% subcategory="Advanced"
    //% group="Remote"
    //% block="Start remote control"
    //% color=#1e90ff
    export function startRemoteControl(): void {
        remoteControlActive = true;
    }

    //% weight=8
    //% subcategory="Advanced"
    //% group="Remote"
    //% block="Stop remote control"
    //% color=#1e90ff
    export function stopRemoteControl(): void {
        remoteControlActive = false;
    }

    function requestPosition(){
        sendMessage(createDobotPacket(10, 0, 0, pins.createBuffer(0)));
    }

    // Communication functions
    let remoteControlCommands = { Start: "START", Stop: "STOP", MoveLinear: "MOVELI", MoveJump: "MOVEJU", JogCartesian: "JOGC", JogJoint: "JOGJ", PumpOff:"PUMPOFF", Grip:"GRIP" };

    // When radio value is received
    radio.onReceivedValue(function (name: string, value: number) {
        if (!remoteControlActive || name==null || value==null) return;

        switch(name)
        {
            case remoteControlCommands.Start:
                moveToStartPosition(PtpMoveMode.Joint);
                break;
            case remoteControlCommands.Stop:
                stopCommand();
                break;
            case remoteControlCommands.MoveLinear:
                moveToFixedPosition(PtpMoveMode.Linear, value);
                break;
            case remoteControlCommands.MoveJump:
                moveToFixedPosition(PtpMoveMode.Jump, value);
                break;
            case remoteControlCommands.JogCartesian:
                moveJog(CoordinateSystem.Cartesian, value);
                break;
            case remoteControlCommands.JogJoint:
                moveJog(CoordinateSystem.Joint, value);
                break;
            case remoteControlCommands.PumpOff:
                setPumpOff();
                break;
            case remoteControlCommands.Grip:
                setGripper(value);
                break;
            default:
        }
    })

    // Setup DOBOT serial connection transmit
    function initConnection(): void {
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

    // Setup DOBOT serial connection receive
    control.inBackground(() => {
        serial.setRxBufferSize(64);
        while(true)
        {
            let buff = serial.readBuffer(0);
            for (let i = 0; i < buff.length; i++) {
                if (buff[i] == 0xAA && buff[i + 1] == 0xAA) {
                    let cmd = buff.getNumber(NumberFormat.UInt8LE, i + 3);

                    switch(cmd)
                    {
                        // Update current position
                        case 10:
                            setCurrentPositionFromBuffer(buff.slice(i + 5, 32));
                        break;
                    }

                }
            }
            
            requestPosition();
            basic.pause(100);
        }
    });

    // Get current position info from buffer data
    function setCurrentPositionFromBuffer(buffer : Buffer)
    {
        let x = buffer.getNumber(NumberFormat.Float32LE, 0);
        let y = buffer.getNumber(NumberFormat.Float32LE, 4);
        let z = buffer.getNumber(NumberFormat.Float32LE, 8);
        let r = buffer.getNumber(NumberFormat.Float32LE, 12);

        let j1 = buffer.getNumber(NumberFormat.Float32LE, 16);
        let j2 = buffer.getNumber(NumberFormat.Float32LE, 20);
        let j3 = buffer.getNumber(NumberFormat.Float32LE, 24);
        let j4 = buffer.getNumber(NumberFormat.Float32LE, 28);

        currentPositionCartesian = new CartesianPosition(x, y, z, r);
        currentPositionJoint = new JointPosition(j1, j2, j3, j4);
    }

    function ProtectCartesianJogCommand(command: JogCommand) : boolean
    {
        let pose = currentPositionCartesian;
        let restrict = false;

        // Check Z-
        if (pose.Z < zMinusLimit && command == JogCommand.CN_DOWN)
        {
            restrict = true;
        }

        // Z+
        if (pose.Z > zPlusLimit && command == JogCommand.CP_DOWN)
        {
            restrict = true;
        }

        // Check X-
        if (pose.X < xMinusLimit && command == JogCommand.AN_DOWN)
        {
            restrict = true;
        }

        // X+
        if (pose.X > xPlusLimit && command == JogCommand.AP_DOWN)
        {
            restrict = true;
        }

        // Check Y-
        if (pose.Y < yMinusLimit && command == JogCommand.BN_DOWN)
        {
            restrict = true;
        }

        // Y-
        if (pose.Y > yPlusLimit && command == JogCommand.BP_DOWN)
        {
            restrict = true;
        }
    
        return restrict;
    }

    // Send message over serial
    function sendMessage(buffer: Buffer) : void
    {
        if (!isInitialised)
        {
            initConnection();
            isInitialised = true;
        }

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