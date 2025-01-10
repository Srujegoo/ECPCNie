#include "F28x_Project.h"
void sysctl(void);
void EPWMInit(void);
int main(void)
float error;
float current_value;
float previous_error;
float output;
float set_value;
float time_change,kp,ki,kd,integral,derivative;
{
    sysctl();

    EPWMInit();

    if(AdcaRegs.ADCINTFLGCLR.bit.ADCINT1==1)
    {
        if(Result[1]>>12)
            {
                EPwm1Regs.CMPA.bit.CMPA=700;


            }
            else if(Result[1]<<12){
                EPwm1Regs.CMPA.bit.CMPA = 400;
            }
        pid_init();


    }
    while(1);




}
void EPWMInit(void)
{
    EALLOW;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1;//clk division enable
    GpioCtrlRegs.GPAMUX1.bit.GPIO0=01;//configuring gpio pin
    GpioCtrlRegs.GPAGMUX1.bit.GPIO0=00;
    GpioCtrlRegs.GPADIR.bit.GPIO0=1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO2=01;//configuring gpio pin
    GpioCtrlRegs.GPAGMUX1.bit.GPIO2=00;
    GpioCtrlRegs.GPADIR.bit.GPIO2=1;
    GpioCtrlRegs.GPAMUX1.bit.GPIO4=01;//configuring gpio pin
    GpioCtrlRegs.GPAGMUX1.bit.GPIO4=00;
    GpioCtrlRegs.GPADIR.bit.GPIO4=1;
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;//enable pwm clk
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM3=1;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=0;//synchronization of the time base

    EPwm1Regs.TBCTL.bit.HSPCLKDIV=2;
    EPwm1Regs.TBPRD=1000;//set the TB period
    EPwm1Regs.TBCTL.bit.CTRMODE=2;//TB mode,counter mode,period
    EPwm1Regs.CMPA.bit.CMPA=700;//compare module,compare register


    EPwm2Regs.TBCTL.bit.HSPCLKDIV=2;
    EPwm2Regs.TBPRD=1000;//set the TB period
    EPwm2Regs.TBCTL.bit.CTRMODE=2;//TB mode,counter mode,period
    EPwm2Regs.CMPA.bit.CMPA=700;//compare module,compare register

    EPwm3Regs.TBCTL.bit.HSPCLKDIV=2;
    EPwm3Regs.TBPRD=1000;//set the TB period
    EPwm3Regs.TBCTL.bit.CTRMODE=2;//TB mode,counter mode,period
    EPwm3Regs.CMPA.bit.CMPA=700;//compare module,compare register

    EPwm1Regs.AQCTLA.bit.ZRO=1;//action qualifier
    EPwm1Regs.AQCTLA.bit.CAU=2;
    EPwm1Regs.TBCTL.bit.PRDLD=0;//prd value is given to shadow register
    //EPwm1Regs.DBCTL.bit.IN_MODE=00;//S4 S5 configuration
    //EPwm1Regs.DBCTL.bit.DEDB_MODE=0;//S8 configuration to zero

    EPwm2Regs.AQCTLA.bit.ZRO=1;//action qualifier
    EPwm2Regs.AQCTLA.bit.CAU=2;
    EPwm2Regs.TBCTL.bit.PRDLD=0;//prd value is given to shadow register
    //EPwm2Regs.DBCTL.bit.IN_MODE=00;//S4 S5 configuration
    //EPwm2Regs.DBCTL.bit.DEDB_MODE=0;//S8 configuration to zero

    EPwm2Regs.AQCTLA.bit.ZRO=1;//action qualifier
    EPwm2Regs.AQCTLA.bit.CAU=2;
    EPwm2Regs.TBCTL.bit.PRDLD=0;//prd value is given to shadow register
    //EPwm2Regs.DBCTL.bit.IN_MODE=00;//S4 S5 configuration
    //EPwm2Regs.DBCTL.bit.DEDB_MODE=0;//S8 configuration to zero

    EPwm1Regs.TBCTL.bit.PHSEN=1;//phase loading enable
    EPwm2Regs.TBCTL.bit.PHSEN=1;
    EPwm3Regs.TBCTL.bit.PHSEN=1;

    EPwm1Regs.TBPHS.bit.TBPHS=500;
    EPwm2Regs.TBPHS.bit.TBPHS=0;
    EPwm3Regs.TBPHS.bit.TBPHS=0;


    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=1;//synchronization of the time base

    EDIS;

}
void adc_init(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1;//Enable the peripheral clock
    //ADC clock has to be given as 15Mhz assume system clk as 160Mhz 160/15 =9.6
       AdcaRegs.ADCCTL2.bit.PRESCALE = 10;

       //ADC  Resolution has to be set
       AdcaRegs.ADCCTL2.bit.RESOLUTION = 0;

       //signal mode has to be specified single / differential
       AdcaRegs.ADCCTL2.bit.SIGNALMODE= 0;

       //ADC power down has to be set 1(active low )
       AdcaRegs.ADCCTL1.bit.ADCPWDNZ= 1;

       //ADC pulse position has to be specified( 0 - raise interrupt at SOC, 1-raise the interrupt at EOC
       AdcaRegs.ADCCTL1.bit.INTPULSEPOS =1;

       //SOC channel select
       AdcaRegs.ADCSOC7CTL.bit.CHSEL=7;
       AdcaRegs.ADCSOC5CTL.bit.CHSEL=5;
       AdcaRegs.ADCSOC2CTL.bit.CHSEL=2;
       AdcaRegs.ADCSOC3CTL.bit.CHSEL=3;

       //SOC trigger select
       AdcaRegs.ADCSOC7CTL.bit.TRIGSEL=3;
       AdcaRegs.ADCSOC5CTL.bit.TRIGSEL=3;
       AdcaRegs.ADCSOC2CTL.bit.TRIGSEL=3;
       AdcaRegs.ADCSOC3CTL.bit.TRIGSEL=3;

       //SOC Acquisition selection (sampling period / system period)-1
       AdcaRegs.ADCSOC4CTL.bit.ACQPS=19;
       AdcaRegs.ADCSOC5CTL.bit.ACQPS=19;
       AdcaRegs.ADCSOC2CTL.bit.ACQPS=19;
       AdcaRegs.ADCSOC3CTL.bit.ACQPS=19;


       //configure ADC interrupt in continuous mode
       AdcaRegs.ADCINTSEL1N2.bit.INT1CONT =1;
       //Select which channel as to be ADC interrupt
       AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 5;
       //ADC interrupt has to be Enabled
       AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;
       //ADC interrupt flag has to be cleared
       EPwm1Regs.CMPA.bit.CMPA=700;
       EDIS;
}
interrupt void ADCISR(void)
{
   Result[0] = AdcaResultRegs.ADCRESULT7; //14 volts in terms of 4096
   Result[1] = Result[0]*0.0034//4096*14/4096
   Result[1]  = AdcaResultRegs.ADCRESULT5;

   CpuTimer1Regs.TCR.bit.TIF=1;
      PieCtrlRegs.PIEACK.bit.ACK1=1;

      //ADC interrupt flag has to be cleared
      AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;
}
   float pid_init(){
       kp =6;
       kd =4 ;
       ki =2 ;
       float error=set_value-current_value;
       integral+=(error*time_change);
       float derivative=(error-previous_error)/time_change;
       output=kp*error+ki*integral+kd*derivative;
       previous_error=error;
   }
































































































































































































































































































































































































































































          ] = AdcaResultRegs.ADCRESULT2;



   CpuTimer1Regs.TCR.bit.TIF=1;
   PieCtrlRegs.PIEACK.bit.ACK1=1;

   //ADC interrupt flag has to be cleared
   AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;
}


    void sysctl(void)

{
    int i;
    EALLOW;
    ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 0;
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0;
    for(i=0;i<5;i++)
    {
        ClkCfgRegs.SYSPLLCTL1.bit.PLLEN = 0;
        ClkCfgRegs.SYSPLLMULT.bit.IMULT = 16;
        ClkCfgRegs.SYSPLLMULT.bit.FMULT = 2;
        while(ClkCfgRegs.SYSPLLSTS.bit.LOCKS !=1);
    }
    ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV = 1;
    ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 1;
    ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV = 0;
    ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0;
    EDIS;
}
