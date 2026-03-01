//% color="#00C04A" weight=100 icon="\uf1b9" block="智能IIC实战巡线"
namespace AnalogLineFollow {
    // ==========================================
    // 全局变量与状态记忆 
    // ==========================================
    let _kp = 0; let _ki = 0; let _kd = 0;
    let _prevError = 0; let _integral = 0;
    let _baseSpeed = 60; let _brake = 1;
    let _integralLimit = 1500; 
    let _lastLeftSpeed = 0; let _lastRightSpeed = 0;
    let _isWhiteLine = false; 
    let _isFirstRun = true;    

    let _leftMotorScale = 1.0;
    let _rightMotorScale = 1.0;

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
        //% block="精准急刹"
        Stop,
        //% block="平滑刹车"
        SmoothBrake,
        //% block="冲过路口(盲开)"
        CrossOver
    }

    export enum SearchStrategy {
        //% block="仅前后探测"
        FrontBack,
        //% block="仅左右探测"
        LeftRight,
        //% block="十字全探测(前后左右)"
        CrossAll
    }

    function _setMotorSpeed(left: number, right: number): void {
        let finalL = Math.max(-100, Math.min(100, left * _leftMotorScale));
        let finalR = Math.max(-100, Math.min(100, right * _rightMotorScale));
        neZha.setMotorSpeed(neZha.MotorList.M1, Math.round(finalL));
        neZha.setMotorSpeed(neZha.MotorList.M2, Math.round(finalR));
    }

    // =================【第一梯队：初始化与校准】=================

    //% block="初始化 IIC巡线 Kp $p Ki $i Kd $d 基础速度 $baseSpeed 刹车 $brake 赛道 $line"
    //% p.defl=0.07 i.defl=0 d.defl=0.09 baseSpeed.defl=60 brake.defl=1
    //% weight=100
    export function setPID(p: number, i: number, d: number, baseSpeed: number, brake: number, line: LineType): void {
        _kp = p; _ki = i; _kd = d; _baseSpeed = baseSpeed; _brake = brake;
        _isWhiteLine = (line === LineType.White);
        _integral = 0; _prevError = 0; _isFirstRun = true; 
    }

    //% block="校准底盘：左轮动力 $left | 右轮动力 $right"
    //% left.defl=100 left.min=50 left.max=100
    //% right.defl=100 right.min=50 right.max=100
    //% weight=99
    export function calibrateMotor(left: number, right: number): void {
        _leftMotorScale = left / 100.0; _rightMotorScale = right / 100.0;
    }

    // =================【第二梯队：核心巡线】=================

    //% block="执行一次PID灰度巡线"
    //% weight=95
    export function pidRun(): void {
        // 🚀 主动获取传感器内置芯片高精度计算后的偏移量
        let error = PlanetX_Basic.TrackBit_get_offset();
        
        if (_isFirstRun) { _prevError = error; _isFirstRun = false; }

        _integral += error;
        _integral = Math.max(-_integralLimit, Math.min(_integralLimit, _integral));

        let derivative = error - _prevError;
        let adjustment = (_kp * error) + (_ki * _integral) + (_kd * derivative);
        _prevError = error;

        let curveSharpness = Math.abs(error) / 100;
        let dynamicBaseSpeed = Math.max(15, _baseSpeed - (curveSharpness * _brake));

        let leftSpeed = dynamicBaseSpeed + adjustment;
        let rightSpeed = dynamicBaseSpeed - adjustment;

        _lastLeftSpeed = leftSpeed; _lastRightSpeed = rightSpeed;
        _setMotorSpeed(leftSpeed, rightSpeed);
    }

    // =================【第三梯队：复杂赛道处理】=================

    //% block="PID巡线 经过 $count 个 $intersectType 后 $action | 冲过速度 $crossSpeed 持续(ms) $crossTime"
    //% count.defl=1 crossSpeed.defl=40 crossTime.defl=300
    //% weight=88
    export function pidCrossMultiple(count: number, intersectType: IntersectType, action: IntersectAction, crossSpeed: number, crossTime: number): void {
        let metCount = 0; 
        while (metCount < count) {
            PlanetX_Basic.Trackbit_get_state_value();
            
            let raw_l2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.One, PlanetX_Basic.TrackbitType.State_1);
            let raw_r2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.Four, PlanetX_Basic.TrackbitType.State_1);

            let l2_on = _isWhiteLine ? raw_l2 : !raw_l2;
            let r2_on = _isWhiteLine ? raw_r2 : !raw_r2;

            let isMet = false;
            if (intersectType === IntersectType.Left) isMet = l2_on;
            else if (intersectType === IntersectType.Right) isMet = r2_on;
            else if (intersectType === IntersectType.Cross) isMet = (l2_on && r2_on);
            else if (intersectType === IntersectType.Any) isMet = (l2_on || r2_on);

            if (isMet) {
                metCount++; 
                if (metCount >= count) {
                    if (action === IntersectAction.Stop) {
                        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0; basic.pause(50); 
                    } else if (action === IntersectAction.SmoothBrake) {
                        smoothBrake(10); 
                    } else if (action === IntersectAction.CrossOver) {
                        _setMotorSpeed(crossSpeed, crossSpeed); basic.pause(crossTime);
                        _lastLeftSpeed = crossSpeed; _lastRightSpeed = crossSpeed;
                    }
                    break; 
                } else {
                    while (true) {
                        pidRun(); 
                        PlanetX_Basic.Trackbit_get_state_value();
                        let check_raw_l2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.One, PlanetX_Basic.TrackbitType.State_1);
                        let check_raw_r2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.Four, PlanetX_Basic.TrackbitType.State_1);

                        let check_l2_on = _isWhiteLine ? check_raw_l2 : !check_raw_l2;
                        let check_r2_on = _isWhiteLine ? check_raw_r2 : !check_raw_r2;

                        let stillMet = false;
                        if (intersectType === IntersectType.Left) stillMet = check_l2_on;
                        else if (intersectType === IntersectType.Right) stillMet = check_r2_on;
                        else if (intersectType === IntersectType.Cross) stillMet = (check_l2_on && check_r2_on);
                        else if (intersectType === IntersectType.Any) stillMet = (check_l2_on || check_r2_on);

                        if (!stillMet) break; 
                        basic.pause(5);
                    }
                }
            } else { 
                // 🚀 神级防扭曲逻辑 (PID 屏蔽区)
                if (intersectType === IntersectType.Cross && (l2_on || r2_on)) {
                    // 当寻找十字/停止线时，如果仅有一个探头踩线，瞬间没收 PID 控制权！
                    // 锁死当前方向，让车子以直线微速滑过最后的临界点，防止“向左/向右猛打方向盘”
                    let lockSpeed = Math.max(20, _baseSpeed * 0.5);
                    _setMotorSpeed(lockSpeed, lockSpeed);
                } else {
                    // 正常巡线区间，让 PID 接管
                    pidRun(); 
                }
                basic.pause(5); 
            }
        }
    }

    //% block="虚线巡线(直/弯通用) 基础速度 $baseSpeed 持续(ms) $timeMs"
    //% baseSpeed.defl=45 timeMs.defl=2000
    //% weight=85
    export function pidDashedLine(baseSpeed: number, timeMs: number): void {
        let endTime = input.runningTime() + timeMs;
        let lastLeft = baseSpeed; let lastRight = baseSpeed;
        let wasLost = true; 

        while (input.runningTime() < endTime) {
            PlanetX_Basic.Trackbit_get_state_value();
            
            let raw_l1 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.Two, PlanetX_Basic.TrackbitType.State_1);
            let raw_r1 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.Three, PlanetX_Basic.TrackbitType.State_1);
            let raw_l2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.One, PlanetX_Basic.TrackbitType.State_1);
            let raw_r2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.Four, PlanetX_Basic.TrackbitType.State_1);

            let l1_on = _isWhiteLine ? raw_l1 : !raw_l1;
            let r1_on = _isWhiteLine ? raw_r1 : !raw_r1;
            let l2_on = _isWhiteLine ? raw_l2 : !raw_l2;
            let r2_on = _isWhiteLine ? raw_r2 : !raw_r2;

            if (l1_on || r1_on || l2_on || r2_on) {
                let error = PlanetX_Basic.TrackBit_get_offset();

                if (wasLost) { _prevError = error; wasLost = false; }

                _integral += error; _integral = Math.max(-_integralLimit, Math.min(_integralLimit, _integral));
                let derivative = error - _prevError;
                let adjustment = (_kp * error) + (_ki * _integral) + (_kd * derivative);
                _prevError = error;

                let leftS = baseSpeed + adjustment;
                let rightS = baseSpeed - adjustment;
                lastLeft = leftS; lastRight = rightS;
                _setMotorSpeed(leftS, rightS);
            } else { 
                wasLost = true; 
                _setMotorSpeed(lastLeft, lastRight); 
            }
            basic.pause(10); 
        }
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
    }

    // =================【第四梯队：智能交互与雷达】=================

    //% block="智能找卡并播报 | 探测模式 $strategy 播报次数 $count 间隔(ms) $interval | 搜索超时(ms) $timeout 搜索速度 $speed 探测步距(ms) $stepTime"
    //% count.defl=2 interval.defl=2000 timeout.defl=10000 speed.defl=40 stepTime.defl=300
    //% weight=75
    export function searchAndReadRFID(strategy: SearchStrategy, count: number, interval: number, timeout: number, speed: number, stepTime: number): void {
        let startTime = input.runningTime();
        let cardFound = false;
        let cardData = "";
        let positionState = 0; 

        function tempMove(lSpeed: number, rSpeed: number, timeMs: number) {
            _setMotorSpeed(lSpeed, rSpeed); basic.pause(timeMs);
            _setMotorSpeed(0, 0); basic.pause(100); 
        }

        while (input.runningTime() - startTime < timeout) {
            if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }

            if (strategy === SearchStrategy.FrontBack || strategy === SearchStrategy.CrossAll) {
                tempMove(speed, speed, stepTime); positionState = 1;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                tempMove(-speed, -speed, stepTime); positionState = 0; 
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                
                tempMove(-speed, -speed, stepTime); positionState = 2;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                tempMove(speed, speed, stepTime); positionState = 0;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
            }

            if (strategy === SearchStrategy.LeftRight || strategy === SearchStrategy.CrossAll) {
                tempMove(-speed, speed, stepTime); positionState = 3;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                tempMove(speed, -speed, stepTime); positionState = 0;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }

                tempMove(speed, -speed, stepTime); positionState = 4;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
                tempMove(-speed, speed, stepTime); positionState = 0;
                if (PlanetX_Basic.checkCard()) { cardFound = true; cardData = PlanetX_Basic.readDataBlock(); break; }
            }
        }

        if (cardFound) {
            control.inBackground(function () {
                for (let i = 0; i < count; i++) {
                    basic.showString(cardData);
                    if (i < count - 1) basic.pause(interval);
                }
                basic.clearScreen();
            });
        } else {
            control.inBackground(function () {
                basic.showIcon(IconNames.No);
                basic.pause(1000);
                basic.clearScreen();
            });
        }

        if (positionState === 1) tempMove(-speed, -speed, stepTime);
        else if (positionState === 2) tempMove(speed, speed, stepTime);
        else if (positionState === 3) tempMove(speed, -speed, stepTime);
        else if (positionState === 4) tempMove(-speed, speed, stepTime);
        
        _setMotorSpeed(0, 0); positionState = 0; 
    }

    // =================【第五梯队：姿态对齐】=================

    //% block="自动对齐停止线(十字/T型) | 调整速度 $speed"
    //% speed.defl=30
    //% weight=65
    export function alignToLine(speed: number): void {
        let alignedCount = 0;
        let timeout = input.runningTime() + 3000;

        while (alignedCount < 3 && input.runningTime() < timeout) {
            PlanetX_Basic.Trackbit_get_state_value();
            
            let raw_l2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.One, PlanetX_Basic.TrackbitType.State_1);
            let raw_r2 = PlanetX_Basic.TrackbitChannelState(PlanetX_Basic.TrackbitChannel.Four, PlanetX_Basic.TrackbitType.State_1);

            let l2_on = _isWhiteLine ? raw_l2 : !raw_l2;
            let r2_on = _isWhiteLine ? raw_r2 : !raw_r2;

            let leftSpeed = 0; let rightSpeed = 0;
            if (!l2_on) leftSpeed = speed;
            if (!r2_on) rightSpeed = speed;

            if (l2_on && r2_on) { alignedCount++; leftSpeed = 0; rightSpeed = 0; } 
            else { alignedCount = 0; }

            _setMotorSpeed(leftSpeed, rightSpeed);
            basic.pause(15);
        }
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0; basic.pause(100);
    }

    //% block="原地死转向 $dir 直到正对线上 | 速度 $speed"
    //% speed.defl=40
    //% weight=60
    export function turnUntilLine(dir: TurnDir, speed: number): void {
        let leftS = dir === TurnDir.Left ? -speed : speed;
        let rightS = dir === TurnDir.Left ? speed : -speed;
        _setMotorSpeed(leftS, rightS); basic.pause(200); 

        while (true) {
            let offset = PlanetX_Basic.TrackBit_get_offset();
            if (Math.abs(offset) < 400) break;
            basic.pause(5);
        }
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0; basic.pause(50);
    }

    // =================【第六梯队：基础运动】=================

    //% block="以 $speed 速度前进 持续(ms) $timeMs"
    //% speed.min=10 speed.max=100 speed.defl=50
    //% timeMs.defl=1000
    //% weight=55
    export function forwardCalibrated(speed: number, timeMs: number): void {
        let safeSpeed = Math.abs(speed); 
        _setMotorSpeed(safeSpeed, safeSpeed);
        _lastLeftSpeed = safeSpeed; _lastRightSpeed = safeSpeed;
        basic.pause(timeMs); 
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
    }

    //% block="以 $speed 速度后退 持续(ms) $timeMs"
    //% speed.min=10 speed.max=100 speed.defl=50
    //% timeMs.defl=1000
    //% weight=54
    export function backwardCalibrated(speed: number, timeMs: number): void {
        let safeSpeed = Math.abs(speed); 
        _setMotorSpeed(-safeSpeed, -safeSpeed);
        _lastLeftSpeed = -safeSpeed; _lastRightSpeed = -safeSpeed;
        basic.pause(timeMs);
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
    }

    //% block="平滑起步/变速 目标速度 $targetSpeed 步进延迟(ms) $delayMs"
    //% targetSpeed.defl=60 delayMs.defl=20
    //% weight=52
    export function smoothStart(targetSpeed: number, delayMs: number): void {
        let currentS = Math.round((_lastLeftSpeed + _lastRightSpeed) / 2);
        let step = (targetSpeed >= currentS) ? 5 : -5;
        for (let s = currentS; (step > 0 ? s <= targetSpeed : s >= targetSpeed); s += step) {
            _setMotorSpeed(s, s); _lastLeftSpeed = s; _lastRightSpeed = s; basic.pause(delayMs);
        }
        _setMotorSpeed(targetSpeed, targetSpeed); _lastLeftSpeed = targetSpeed; _lastRightSpeed = targetSpeed;
    }

    //% block="平滑刹车 步进延迟(ms) $delayMs"
    //% delayMs.defl=20
    //% weight=50
    export function smoothBrake(delayMs: number): void {
        let steps = 10;
        let leftStep = _lastLeftSpeed / steps; let rightStep = _lastRightSpeed / steps;
        for (let i = 0; i < steps; i++) {
            _lastLeftSpeed -= leftStep; _lastRightSpeed -= rightStep;
            _setMotorSpeed(_lastLeftSpeed, _lastRightSpeed); basic.pause(delayMs);
        }
        _setMotorSpeed(0, 0); _lastLeftSpeed = 0; _lastRightSpeed = 0;
    }

    // 🚀 新增：一键紧急停止所有电机
    //% block="停止所有电机"
    //% weight=45
    export function stopMotors(): void {
        _setMotorSpeed(0, 0);
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
    }
}
