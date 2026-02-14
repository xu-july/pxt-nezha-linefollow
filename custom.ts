//% color="#00C04A" weight=100 icon="\uf1b9" block="智能IIC极简巡线"
namespace AnalogLineFollow {
    let _kp = 0;
    let _ki = 0;
    let _kd = 0;
    let _prevError = 0;
    let _integral = 0;

    let _baseSpeed = 60;
    let _brake = 1;

    // 隐藏的内部阈值：因为硬件有一键学习功能，数值会两极分化，固定150极其安全
    let _internalThreshold = 150;

    let _lastLeftSpeed = 0;
    let _lastRightSpeed = 0;

    let _isWhiteLine = false;

    export enum TurnDir {
        //% block="左"
        Left,
        //% block="右"
        Right
    }

    export enum LineType {
        //% block="黑线(白底)"
        Black,
        //% block="白线(黑底)"
        White
    }

    export enum IntersectType {
        //% block="左路口"
        Left,
        //% block="右路口"
        Right,
        //% block="十字/停止线"
        Cross,
        //% block="任意路口"
        Any
    }

    export enum IntersectAction {
        //% block="平滑停车"
        Stop,
        //% block="冲过路口(盲开)"
        CrossOver
    }

    // 🚀 删除了阈值输入框，界面极致清爽！
    //% block="初始化 IIC巡线 Kp $p Ki $i Kd $d 基础速度 $baseSpeed 刹车 $brake 赛道 $line"
    //% p.defl=0.07 i.defl=0 d.defl=0.09 baseSpeed.defl=60 brake.defl=1
    //% weight=100
    export function setPID(p: number, i: number, d: number, baseSpeed: number, brake: number, line: LineType): void {
        _kp = p;
        _ki = i;
        _kd = d;
        _baseSpeed = baseSpeed;
        _brake = brake;
        _isWhiteLine = (line === LineType.White);
        _integral = 0;
        _prevError = 0;
    }

    //% block="平滑起步/变速 目标速度 $targetSpeed 步进延迟(ms) $delayMs"
    //% targetSpeed.defl=80 delayMs.defl=20
    //% weight=90
    export function smoothStart(targetSpeed: number, delayMs: number): void {
        let currentS = Math.round((_lastLeftSpeed + _lastRightSpeed) / 2);
        let step = (targetSpeed >= currentS) ? 5 : -5;

        for (let s = currentS; (step > 0 ? s <= targetSpeed : s >= targetSpeed); s += step) {
            neZha.setMotorSpeed(neZha.MotorList.M1, s);
            neZha.setMotorSpeed(neZha.MotorList.M2, s);
            _lastLeftSpeed = s;
            _lastRightSpeed = s;
            basic.pause(delayMs);
        }
        neZha.setMotorSpeed(neZha.MotorList.M1, targetSpeed);
        neZha.setMotorSpeed(neZha.MotorList.M2, targetSpeed);
        _lastLeftSpeed = targetSpeed;
        _lastRightSpeed = targetSpeed;
    }

    //% block="平滑刹车 步进延迟(ms) $delayMs"
    //% delayMs.defl=20
    //% weight=80
    export function smoothBrake(delayMs: number): void {
        let steps = 10;
        let leftStep = _lastLeftSpeed / steps;
        let rightStep = _lastRightSpeed / steps;

        for (let i = 0; i < steps; i++) {
            _lastLeftSpeed -= leftStep;
            _lastRightSpeed -= rightStep;
            neZha.setMotorSpeed(neZha.MotorList.M1, _lastLeftSpeed);
            neZha.setMotorSpeed(neZha.MotorList.M2, _lastRightSpeed);
            basic.pause(delayMs);
        }
        neZha.setMotorSpeed(neZha.MotorList.M1, 0);
        neZha.setMotorSpeed(neZha.MotorList.M2, 0);
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
    }

    //% block="原地向 $dir 转，直到识别到状态 $targetState | 速度 $speed"
    //% speed.defl=40
    //% weight=75
    export function turnUntilState(dir: TurnDir, targetState: PlanetX_Basic.TrackbitStateType, speed: number): void {
        let leftS = dir === TurnDir.Left ? -speed : speed;
        let rightS = dir === TurnDir.Left ? speed : -speed;

        neZha.setMotorSpeed(neZha.MotorList.M1, leftS);
        neZha.setMotorSpeed(neZha.MotorList.M2, rightS);
        basic.pause(200);

        while (true) {
            PlanetX_Basic.Trackbit_get_state_value();
            if (PlanetX_Basic.TrackbitState(targetState)) {
                break;
            }
            basic.pause(5);
        }
        neZha.setMotorSpeed(neZha.MotorList.M1, 0);
        neZha.setMotorSpeed(neZha.MotorList.M2, 0);
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
        basic.pause(50);
    }

    //% block="PID巡线 直到遇见 $intersectType 然后 $action | 冲过速度 $crossSpeed 持续(ms) $crossTime"
    //% crossSpeed.defl=40 crossTime.defl=300
    //% weight=72
    export function pidUntilIntersection(intersectType: IntersectType, action: IntersectAction, crossSpeed: number, crossTime: number): void {
        while (true) {
            PlanetX_Basic.Trackbit_get_state_value();

            // 依赖硬件学习后的极度可靠数据
            let l2 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.One);
            let r2 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.Four);

            let l2_on = _isWhiteLine ? (l2 > _internalThreshold) : (l2 < _internalThreshold);
            let r2_on = _isWhiteLine ? (r2 > _internalThreshold) : (r2 < _internalThreshold);

            let isMet = false;
            // 完全符合你的要求：1号看白即左，4号看白即右
            if (intersectType === IntersectType.Left) isMet = l2_on;
            else if (intersectType === IntersectType.Right) isMet = r2_on;
            else if (intersectType === IntersectType.Cross) isMet = (l2_on && r2_on);
            else if (intersectType === IntersectType.Any) isMet = (l2_on || r2_on);

            if (isMet) {
                if (action === IntersectAction.Stop) {
                    smoothBrake(10);
                }
                else if (action === IntersectAction.CrossOver) {
                    neZha.setMotorSpeed(neZha.MotorList.M1, crossSpeed);
                    neZha.setMotorSpeed(neZha.MotorList.M2, crossSpeed);
                    basic.pause(crossTime);
                    _lastLeftSpeed = crossSpeed;
                    _lastRightSpeed = crossSpeed;
                }
                break;
            }

            pidRun();
            basic.pause(5);
        }
    }

    //% block="自动对齐停止线(十字/T型) | 调整速度 $speed"
    //% speed.defl=30
    //% weight=71
    export function alignToLine(speed: number): void {
        let alignedCount = 0;
        let timeout = input.runningTime() + 3000;

        while (alignedCount < 3 && input.runningTime() < timeout) {
            PlanetX_Basic.Trackbit_get_state_value();
            let l2 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.One);
            let r2 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.Four);

            let l2_on = _isWhiteLine ? (l2 > _internalThreshold) : (l2 < _internalThreshold);
            let r2_on = _isWhiteLine ? (r2 > _internalThreshold) : (r2 < _internalThreshold);

            let leftSpeed = 0;
            let rightSpeed = 0;

            if (!l2_on) leftSpeed = speed;
            if (!r2_on) rightSpeed = speed;

            if (l2_on && r2_on) {
                alignedCount++;
                leftSpeed = 0;
                rightSpeed = 0;
            } else {
                alignedCount = 0;
            }

            neZha.setMotorSpeed(neZha.MotorList.M1, leftSpeed);
            neZha.setMotorSpeed(neZha.MotorList.M2, rightSpeed);
            basic.pause(15);
        }

        neZha.setMotorSpeed(neZha.MotorList.M1, 0);
        neZha.setMotorSpeed(neZha.MotorList.M2, 0);
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
        basic.pause(100);
    }

    // ==========================================
    // 🚀 IIC 高精度偏移量 PID 核心引擎
    // ==========================================
    //% block="执行一次PID灰度巡线"
    //% weight=70
    export function pidRun(): void {
        let error = PlanetX_Basic.TrackBit_get_offset();

        if (_isWhiteLine) {
            error = -error;
        }

        _integral += error;
        let derivative = error - _prevError;

        let adjustment = (_kp * error) + (_ki * _integral) + (_kd * derivative);

        _prevError = error;

        let curveSharpness = Math.abs(error) / 100;
        let dynamicBaseSpeed = _baseSpeed - (curveSharpness * _brake);
        dynamicBaseSpeed = Math.max(15, dynamicBaseSpeed);

        let leftSpeed = dynamicBaseSpeed + adjustment;
        let rightSpeed = dynamicBaseSpeed - adjustment;

        leftSpeed = Math.max(-100, Math.min(100, leftSpeed));
        rightSpeed = Math.max(-100, Math.min(100, rightSpeed));

        _lastLeftSpeed = leftSpeed;
        _lastRightSpeed = rightSpeed;

        neZha.setMotorSpeed(neZha.MotorList.M1, leftSpeed);
        neZha.setMotorSpeed(neZha.MotorList.M2, rightSpeed);
    }
}
