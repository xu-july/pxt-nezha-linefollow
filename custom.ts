//% color="#00C04A" weight=100 icon="\uf1b9" block="模拟灰度巡线控制"
namespace AnalogLineFollow {
    let _kp = 0;
    let _ki = 0;
    let _kd = 0;
    let _prevError = 0;
    let _integral = 0;

    let _baseSpeed = 40;
    let _brake = 0;
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

    //% block="初始化 Kp $p Ki $i Kd $d 基础速度 $baseSpeed 刹车 $brake 赛道 $line"
    //% p.defl=1.5 i.defl=0 d.defl=0.8 baseSpeed.defl=40 brake.defl=5
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
        }

        neZha.setMotorSpeed(neZha.MotorList.M1, 0);
        neZha.setMotorSpeed(neZha.MotorList.M2, 0);
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
        basic.pause(50);
    }

    //% block="执行一次PID灰度巡线"
    //% weight=70
    export function pidRun(): void {
        // 1. 获取所有探头模拟值
        let l2_val = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.One);
        let l1_val = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.Two);
        let r1_val = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.Three);
        let r2_val = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.Four);

        let isLost = false;

        // 2. 🌟 黑科技：独立阈值检测"断线/虚线"状态
        // 探头读数：遇到黑线数值小(<100)，遇到白纸数值大(>150)
        if (_isWhiteLine) {
            // 巡白线：如果4个探头全是黑底（读数均小于100），说明进入虚线空隙
            if (l2_val < 100 && l1_val < 100 && r1_val < 100 && r2_val < 100) {
                isLost = true;
            }
        } else {
            // 巡黑线：如果4个探头全是白底（读数均大于150），说明进入虚线空隙
            if (l2_val > 150 && l1_val > 150 && r1_val > 150 && r2_val > 150) {
                isLost = true;
            }
        }

        // 3. 断线续航（姿态保持系统）触发
        if (isLost) {
            // 冻结 PID，直接按照脱线前最后一毫秒的车身姿态（速度差）进行盲开滑行！
            neZha.setMotorSpeed(neZha.MotorList.M1, _lastLeftSpeed);
            neZha.setMotorSpeed(neZha.MotorList.M2, _lastRightSpeed);
            // 直接 return 结束本次循环，不更新 _prevError，保证找到线后无缝衔接
            return;
        }

        // --- 以下为正常的 PID 差速计算 ---
        let left_weight = (l2_val * 2) + l1_val;
        let right_weight = (r2_val * 2) + r1_val;
        let error = left_weight - right_weight;

        error = error / 100;

        if (_isWhiteLine) {
            error = -error;
        }

        _integral += error;
        let derivative = error - _prevError;
        let adjustment = (_kp * error) + (_ki * _integral) + (_kd * derivative);

        _prevError = error;

        let curveSharpness = Math.abs(error);
        let dynamicBaseSpeed = _baseSpeed - (curveSharpness * _brake);
        dynamicBaseSpeed = Math.max(10, dynamicBaseSpeed);

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
