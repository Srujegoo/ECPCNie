#include "F28x_Project.h"
#include "math.h"
void sysctl(void);
void EPWM(void);
__interrupt void epwm1ISR(void);
__interrupt void epwm2ISR(void);
__interrupt void epwm3ISR(void);


//void ADCA_Init(void);
//float pid_init();
//void SysClk(void);
//void InitTimer0(void);
//void QEP(void);
//void timer1(void);
//__interrupt void ADCISR(void);
//__interrupt void Timer0ISR(void);
//__interrupt void EQEPisr(void);
//int Digital_Result[5];
//int a[5];
//int pid[5];
//int u;
//float v, w,y;
//int y;


int t = 0;
float duty;
int lkb[5];


double pi  =  3.141592653589793;

void main(void)
{

sysctl();
EPWM();
DINT;
      IER=00;
      EALLOW;
      PieCtrlRegs.PIECTRL.bit.ENPIE=1; // Enable the PIE.
      PieVectTable.EPWM1_INT=&epwm1ISR ;//  16 channels 12 groups
      PieVectTable.EPWM2_INT=&epwm2ISR ;
      PieVectTable.EPWM3_INT=&epwm3ISR ;
      PieCtrlRegs.PIEIER3.bit.INTx1 = 1;
      PieCtrlRegs.PIEIER3.bit.INTx2 =  1;
      PieCtrlRegs.PIEIER3.bit.INTx3 = 1;

      PieCtrlRegs.PIEACK.bit.ACK3 = 1;
      IER=0xffff;
      EINT;
      EDIS;

}
void sysctl(void)
    {
        int i;
        EALLOW;
        ClkCfgRegs.CLKSRCCTL1.bit.OSCCLKSRCSEL = 0 ;  // INTOSC 2 i am using - 10MHZ
        ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 0 ;     //  PLLCLKEN
        for(i=0;i<5;i++)
        {
            ClkCfgRegs.SYSPLLCTL1.bit.PLLEN = 0 ;         // i need to  get 1khz///
            ClkCfgRegs.SYSPLLMULT.bit.IMULT = 16 ;        //integer multiplier factor.
            ClkCfgRegs.SYSPLLMULT.bit.FMULT = 4;         // fractional multiplier factor. 80mhz
            while(ClkCfgRegs.SYSPLLSTS.bit.LOCKS !=1);
        }
        ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV = 0;  //40mhz
        ClkCfgRegs.SYSPLLCTL1.bit.PLLCLKEN = 1 ;
        ClkCfgRegs.SYSCLKDIVSEL.bit.PLLSYSCLKDIV = 5 ;  //4MHZ
        ClkCfgRegs.LOSPCP.bit.LSPCLKDIV = 0 ;
        EDIS;
    }




void EPWM(void){
      EALLOW;


       ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=0;// EPWM clock - System clk division   SYSCLK/1
       GpioCtrlRegs.GPAMUX1.bit.GPIO0=01;//configuring gpio pin
       GpioCtrlRegs.GPAGMUX1.bit.GPIO0=00;

       GpioCtrlRegs.GPADIR.bit.GPIO0=01;
       GpioCtrlRegs.GPAMUX1.bit.GPIO2=01;//configuring gpio pin
       GpioCtrlRegs.GPAGMUX1.bit.GPIO2=00;
       GpioCtrlRegs.GPADIR.bit.GPIO2=1;

       GpioCtrlRegs.GPAMUX1.bit.GPIO4=01;//configuring gpio pin
       GpioCtrlRegs.GPAGMUX1.bit.GPIO4=00;
       GpioCtrlRegs.GPADIR.bit.GPIO4=01;

       CpuSysRegs.PCLKCR2.bit.EPWM2=1;//enable pwm clk   //40 pin
       CpuSysRegs.PCLKCR2.bit.EPWM1=1;                   //38 pin
       CpuSysRegs.PCLKCR2.bit.EPWM3=1;                   //36 pin
       CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=0;//synchronization of the time base

       EPwm1Regs.TBCTL.bit.HSPCLKDIV=2;
       EPwm1Regs.TBPRD=2000;//set the TB period
       EPwm1Regs.TBCTL.bit.CTRMODE=3;
       EPwm1Regs.CMPA.bit.CMPA = (int)duty;

      EPwm2Regs.TBCTL.bit.HSPCLKDIV=2;  //SYSCLK/2 = 2MHZ
      EPwm2Regs.TBPRD=2000;//set the TB period                    17.6 = 960
      EPwm2Regs.TBCTL.bit.CTRMODE=2;//TB mode, up downcounter mode,period 18.2 =              810
      EPwm2Regs.CMPA.bit.CMPA = (int)duty ;//re module,compare register // 19.2 volts =  710 = 40
                                                                  //  20 volts           = 500 =  50                                       //  22.4volts                    = 350 = 60
         EPwm3Regs.TBCTL.bit.HSPCLKDIV=2;                            // 28.8   = 70,210
         EPwm3Regs.TBPRD=2000;//set the TB period  2MHZ/2000 = 1000HZ     // 42 v = 80 = 120
         EPwm3Regs.TBCTL.bit.CTRMODE=2;//TB mode,counter mode,period
         EPwm3Regs.CMPA.bit.CMPA = (int)duty;//compare module,compare register

         EPwm1Regs.AQCTLA.bit.ZRO=0;//action qualifier
         EPwm1Regs.AQCTLA.bit.CAU=3;

         EPwm2Regs.AQCTLA.bit.ZRO=0;//action qualifier
         EPwm2Regs.AQCTLA.bit.CAU=3;

         EPwm3Regs.AQCTLA.bit.ZRO=0;//action qualifier
         EPwm3Regs.AQCTLA.bit.CAU=3;



         EPwm1Regs.TBCTL.bit.PHSEN=1;//phase loading enable
         EPwm2Regs.TBCTL.bit.PHSEN=1;
         EPwm3Regs.TBCTL.bit.PHSEN=1;

         EPwm1Regs.TBPHS.bit.TBPHS= 1233;
         EPwm2Regs.TBPHS.bit.TBPHS= 667;
         EPwm3Regs.TBPHS.bit.TBPHS = 0;

        EPwm3Regs.ETSEL.bit.INTSELCMP = 0;// compared with cmpa interrupt should generate.

        EPwm1Regs.ETSEL.bit.INTEN = 1; // interrupt enable.
        EPwm1Regs.ETSEL.bit.INTSEL = 4;
        EPwm1Regs.ETCLR.bit.INT =  1;

        EPwm2Regs.ETSEL.bit.INTEN = 1; // interrupt enable.
        EPwm2Regs.ETSEL.bit.INTSEL = 5;
        EPwm2Regs.ETCLR.bit.INT =  1;

        EPwm3Regs.ETSEL.bit.INTEN = 1; // interrupt enable.
        EPwm3Regs.ETSEL.bit.INTSEL = 1;
        EPwm3Regs.ETCLR.bit.INT =  1;


        CpuSysRegs.PCLKCR0.bit.TBCLKSYNC=1;


}

__interrupt void epwm1ISR(void){
   EALLOW;

      t++;

      if (t >= 360) {
          t = 0;
           }
           float Mod_index = 0.5;
          duty = 1000*(1 + Mod_index*sin(2*pi*t/360));

           EPwm1Regs.ETCLR.bit.INT =  1;

           PieCtrlRegs.PIEACK.bit.ACK3 = 1;

   EDIS;


       }


__interrupt void epwm2ISR(void){
   EALLOW;

      t++;

      if (t >= 360) {
          t = 0;
           }
           float Mod_index = 0.5;
          duty = 1000*(1 + Mod_index*sin(2*pi*t/360 + 120));


           EPwm2Regs.ETCLR.bit.INT =  1;

           PieCtrlRegs.PIEACK.bit.ACK3 = 1;

   EDIS;


       }



__interrupt void epwm3ISR(void){
   EALLOW;

      t++;

      if (t >= 360) {
          t = 0;
           }
           float Mod_index = 0.5;
          duty = 1000*(1 + Mod_index*sin(2*pi*t/360));

           EPwm3Regs.ETCLR.bit.INT =  1;
           PieCtrlRegs.PIEACK.bit.ACK3 = 1;

   EDIS;


       }



