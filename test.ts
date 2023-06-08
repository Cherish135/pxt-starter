// 在此处测试；当此软件包作为插件使用时，将不会编译此软件包。
let num = 0
basic.forever(function () {
    if (BanBao.oneOfFiveWayInfraredDigitalRead(I2CPort.S4, InfraredChoose.L1, InfraredMode.White)) {
        num = 1
    } else if (BanBao.oneOfFiveWayInfraredDigitalRead(I2CPort.S4, InfraredChoose.L2, InfraredMode.White)) {
        num = 2
    } else if (BanBao.oneOfFiveWayInfraredDigitalRead(I2CPort.S4, InfraredChoose.M, InfraredMode.White)) {
        num = 3
    } else if (BanBao.oneOfFiveWayInfraredDigitalRead(I2CPort.S4, InfraredChoose.R1, InfraredMode.White)) {
        num = 4
    } else if (BanBao.oneOfFiveWayInfraredDigitalRead(I2CPort.S4, InfraredChoose.R2, InfraredMode.White)) {
        num = 5
    } else if (input.buttonIsPressed(Button.A)) {
        num = 0
    }
    if (num == 1) {
        BanBao.motorRotation(Port.S1, 100, MotorDirection.Foreward)
    } else if (num == 2) {
        BanBao.motorRotation(Port.S1, 25, MotorDirection.Reversal)
    } else if (num == 3) {
        BanBao.motorRotation(Port.S2, 100, MotorDirection.Reversal)
    } else if (num == 4) {
        BanBao.motorRotation(Port.S2, 25, MotorDirection.Foreward)
    } else if (num == 5) {
        basic.showString("" + (BanBao.ultrasonicDistance(Port.S3)))
    } else if (num == 0) {
        BanBao.motorStop(Port.S1, MotorStopMode.Gliding)
        BanBao.motorStop(Port.S2, MotorStopMode.Brake)
    }
})
