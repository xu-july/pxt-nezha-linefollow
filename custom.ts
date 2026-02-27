//% color="#00C04A" weight=100 icon="\uf1b9" block="智能IIC实战巡线"
namespace AnalogLineFollow {
    // PID 与基础参数
    let _kp = 0;
    let _ki = 0;
    let _kd = 0;
    let _prevError = 0;
    let _integral = 0;
    let _baseSpeed = 60;
    let _brake = 1;
    let _internalThreshold = 150;

    // 状态记忆
    let _lastLeftSpeed = 0;
    let _lastRightSpeed = 0;
    let _isWhiteLine = false;

    // 🚀 底盘硬件校准系数（默认1.0，即100%动力）
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

    // ==========================================
    // 🚀 核心底层：所有速度指令必须经过此拦截器！
    // ==========================================
    function _setMotorSpeed(left: number, right: number): void {
        let finalL = left * _leftMotorScale;
        let finalR = right * _rightMotorScale;

        // 限制输出在 -100 到 100 之间，防止数值爆炸
        finalL = Math.max(-100, Math.min(100, finalL));
        finalR = Math.max(-100, Math.min(100, finalR));

        neZha.setMotorSpeed(neZha.MotorList.M1, Math.round(finalL));
        neZha.setMotorSpeed(neZha.MotorList.M2, Math.round(finalR));
    }

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

    // 🚀 实战积木 3：底盘硬件校准 (完美修复输入框Bug)
    //% block="校准底盘：左轮动力 $left | 右轮动力 $right"
    //% left.defl=100 left.min=50 left.max=100
    //% right.defl=100 right.min=50 right.max=100
    //% weight=95
    export function calibrateMotor(left: number, right: number): void {
        _leftMotorScale = left / 100.0;
        _rightMotorScale = right / 100.0;
    }

    // 🚀 实战积木 4：精准校准前进 (带自动刹车)
    //% block="以 $speed 速度前进 持续(ms) $timeMs"
    //% speed.min=10 speed.max=100 speed.defl=50
    //% timeMs.defl=1000
    //% weight=94
    export function forwardCalibrated(speed: number, timeMs: number): void {
        let safeSpeed = Math.abs(speed); // 防呆：防止误填负数
        
        // 直接调用底层拦截器，它会自动帮你乘上底盘校准比例！
        _setMotorSpeed(safeSpeed, safeSpeed);
        
        // 同步状态记忆，方便后续如果接平滑刹车能读取到正确速度
        _lastLeftSpeed = safeSpeed;
        _lastRightSpeed = safeSpeed;
        
        basic.pause(timeMs); // 持续运行设定的时间
        
        // 跑完瞬间刹车，并清零速度记忆
        _setMotorSpeed(0, 0); 
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
    }

    // 🚀 实战积木 5：精准校准后退 (带自动刹车)
    //% block="以 $speed 速度后退 持续(ms) $timeMs"
    //% speed.min=10 speed.max=100 speed.defl=50
    //% timeMs.defl=1000
    //% weight=93
    export function backwardCalibrated(speed: number, timeMs: number): void {
        let safeSpeed = Math.abs(speed); 
        
        // 后退就是加上负号，底层依然会完美按比例分配负电压！
        _setMotorSpeed(-safeSpeed, -safeSpeed);
        _lastLeftSpeed = -safeSpeed;
        _lastRightSpeed = -safeSpeed;
        
        basic.pause(timeMs);
        
        _setMotorSpeed(0, 0); 
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
    }

    //% block="平滑起步/变速 目标速度 $targetSpeed 步进延迟(ms) $delayMs"
    //% targetSpeed.defl=60 delayMs.defl=20
    //% weight=90
    export function smoothStart(targetSpeed: number, delayMs: number): void {
        let currentS = Math.round((_lastLeftSpeed + _lastRightSpeed) / 2);
        let step = (targetSpeed >= currentS) ? 5 : -5;

        for (let s = currentS; (step > 0 ? s <= targetSpeed : s >= targetSpeed); s += step) {
            _setMotorSpeed(s, s);
            _lastLeftSpeed = s;
            _lastRightSpeed = s;
            basic.pause(delayMs);
        }
        _setMotorSpeed(targetSpeed, targetSpeed);
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
            _setMotorSpeed(_lastLeftSpeed, _lastRightSpeed);
            basic.pause(delayMs);
        }
        _setMotorSpeed(0, 0);
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
    }

    // 🚀 实战积木 2：智能原地死转直到线上
    //% block="原地死转向 $dir 直到正对线上 | 速度 $speed"
    //% speed.defl=40
    //% weight=75
    export function turnUntilLine(dir: TurnDir, speed: number): void {
        let leftS = dir === TurnDir.Left ? -speed : speed;
        let rightS = dir === TurnDir.Left ? speed : -speed;

        _setMotorSpeed(leftS, rightS);
        basic.pause(200); // 先盲转0.2秒，强行脱离当前压着的黑线

        while (true) {
            // 调用 V8 引擎的高精度偏移量，只要偏差在 -400 到 400 之间，说明车头已经完美正对黑线！
            let offset = PlanetX_Basic.TrackBit_get_offset();
            if (Math.abs(offset) < 400) {
                break;
            }
            basic.pause(5);
        }
        _setMotorSpeed(0, 0); // 瞬间死刹
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
        basic.pause(50);
    }

    // 🚀 实战积木 1：万能路口计数器 (支持多选项卡)
    //% block="PID巡线 经过 $count 个 $intersectType 后 $action | 冲过速度 $crossSpeed 持续(ms) $crossTime"
    //% count.defl=1 crossSpeed.defl=40 crossTime.defl=300
    //% weight=73
    export function pidCrossMultiple(count: number, intersectType: IntersectType, action: IntersectAction, crossSpeed: number, crossTime: number): void {
        let metCount = 0; // 记录遇到了几个路口

        while (metCount < count) {
            PlanetX_Basic.Trackbit_get_state_value();
            let l2 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.One);
            let r2 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.Four);

            let l2_on = _isWhiteLine ? (l2 > _internalThreshold) : (l2 < _internalThreshold);
            let r2_on = _isWhiteLine ? (r2 > _internalThreshold) : (r2 < _internalThreshold);

            let isMet = false;
            if (intersectType === IntersectType.Left) isMet = l2_on;
            else if (intersectType === IntersectType.Right) isMet = r2_on;
            else if (intersectType === IntersectType.Cross) isMet = (l2_on && r2_on);
            else if (intersectType === IntersectType.Any) isMet = (l2_on || r2_on);

            if (isMet) {
                metCount++; // 发现目标路口，计数+1

                if (metCount >= count) {
                    // 🚀 如果数量达标，根据选择执行对应的动作
                    if (action === IntersectAction.Stop) {
                        _setMotorSpeed(0, 0); 
                        _lastLeftSpeed = 0;
                        _lastRightSpeed = 0;
                        basic.pause(50); // 瞬间死刹并稍微锁死一瞬间，彻底消除物理惯性
                    } else if (action === IntersectAction.SmoothBrake) {
                        smoothBrake(10); // 启动平滑刹车（每步延迟10毫秒，用时约100毫秒温柔停下）
                    } else if (action === IntersectAction.CrossOver) {
                        _setMotorSpeed(crossSpeed, crossSpeed);
                        basic.pause(crossTime);
                        _lastLeftSpeed = crossSpeed;
                        _lastRightSpeed = crossSpeed;
                    }
                    break; // 彻底结束这个方块
                } else {
                    let passSpeed = Math.max(35, _baseSpeed);
                    _setMotorSpeed(passSpeed, passSpeed);
                    basic.pause(300); // 冷却时间 (跨越路口防抖)
                }
            } else {
                pidRun(); // 没遇到路口就正常巡线
                basic.pause(5);
            }
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

            _setMotorSpeed(leftSpeed, rightSpeed);
            basic.pause(15);
        }

        _setMotorSpeed(0, 0);
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
        basic.pause(100);
    }

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

        // 这里只限制逻辑计算的速度
        leftSpeed = Math.max(-100, Math.min(100, leftSpeed));
        rightSpeed = Math.max(-100, Math.min(100, rightSpeed));

        _lastLeftSpeed = leftSpeed;
        _lastRightSpeed = rightSpeed;

        // 实际输出依然会被底层的校准拦截器处理
        _setMotorSpeed(leftSpeed, rightSpeed);
    }
        // 🚀 实战积木 6：智能虚线巡线 (失线记忆法)
    //% block="虚线巡线(直/弯通用) 基础速度 $baseSpeed 持续(ms) $timeMs"
    //% baseSpeed.defl=45 timeMs.defl=2000
    //% weight=68
    export function pidDashedLine(baseSpeed: number, timeMs: number): void {
        let endTime = input.runningTime() + timeMs;
        
        // 创建“记忆库”：默认起步全速前进
        let lastLeft = baseSpeed;
        let lastRight = baseSpeed;

        while (input.runningTime() < endTime) {
            PlanetX_Basic.Trackbit_get_state_value();
            
            // 读取四个探头的灰度值，判断是否能看到线
            let l1 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.Two);
            let r1 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.Three);
            let l2 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.One);
            let r2 = PlanetX_Basic.TrackbitgetGray(PlanetX_Basic.TrackbitChannel.Four);

            let l1_on = _isWhiteLine ? (l1 > _internalThreshold) : (l1 < _internalThreshold);
            let r1_on = _isWhiteLine ? (r1 > _internalThreshold) : (r1 < _internalThreshold);
            let l2_on = _isWhiteLine ? (l2 > _internalThreshold) : (l2 < _internalThreshold);
            let r2_on = _isWhiteLine ? (r2 > _internalThreshold) : (r2 < _internalThreshold);

            // 只要有任何一个探头能看到线，就认为“在线上”
            let hasLine = l1_on || r1_on || l2_on || r2_on;

            if (hasLine) {
                // --- 1. 在线上：正常PID巡线，并更新记忆库 ---
                let error = PlanetX_Basic.TrackBit_get_offset();
                if (_isWhiteLine) error = -error;

                _integral += error;
                let derivative = error - _prevError;
                let adjustment = (_kp * error) + (_ki * _integral) + (_kd * derivative);
                _prevError = error;

                let leftS = baseSpeed + adjustment;
                let rightS = baseSpeed - adjustment;

                // 限制在-100到100之间防爆表
                leftS = Math.max(-100, Math.min(100, leftS));
                rightS = Math.max(-100, Math.min(100, rightS));

                // 关键一步：把算出来的速度存入记忆库！
                lastLeft = leftS;
                lastRight = rightS;

                _setMotorSpeed(leftS, rightS);
            } else {
                // --- 2. 瞎了(在空白处)：进入盲开记忆模式 ---
                // 停止计算，直接输出断线前最后一瞬间保存的速度！
                _setMotorSpeed(lastLeft, lastRight);
            }
            
            basic.pause(10); // 10毫秒刷新一次状态
        }
        
        // 设定的时间到了，跑完这段虚线，执行瞬间死刹
        _setMotorSpeed(0, 0);
        _lastLeftSpeed = 0;
        _lastRightSpeed = 0;
    }
}
