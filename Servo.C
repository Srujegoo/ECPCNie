 void EQep(void)
            {


             CpuSysRegs.PCLKCR4.bit.EQEP1 = 01 // enable the clock
             CpuSysRegs.PCLKCR4.bit.EQEP2 = 01 // enable the clock



             GpioCtrlRegs.GPAGMUX2.bit.GPIO20 = 00;
             GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 01;
             GpioCtrlRegs.GPADIR.bit.GPIO20 = 00;

             GpioCtrlRegs.GPAGMUX2.bit.GPIO21 = 00;
             GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 01;
             GpioCtrlRegs.GPADIR.bit.GPIO21 = 00;

             EQep1Regs.QDECCTL.bit.QSRC = 0; //Quadrature count mode..
             EQep1Regs.QPOSINIT = p
             EQep1Regs.QPOSMAX = q;
             if(EQep1Regs.QPOSCNT == p ){
             EQep1Regs.QUTMR = 01;
}
             else if(EQep1Regs.QPOSCNT == q){
             


}
