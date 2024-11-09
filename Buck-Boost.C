#include "F28x_Project.h"
void sysctl(void);
void EPWMInit(void);
//void EPWMISR(void);
void ADCA_Init(void);
float pid_init();
void SysClk(void);
void InitTimer0(void);
void timer1(void);
__interrupt void ADCISR(void);
__interrupt void Timer0ISR(void);
int  Digital_Result[5];
int a[5];
float Kp;
float Ki;
float Kd;
float previous_error;
float V_ref = 3156.142;
int V_out;
float integral;
float derivative;
float error;
//float previous_error;
float dt;
float pid1_output;
//float pid2_output;
float pid_output;
//int pid[5];
//int u;
//float v, w,y;
int y;
int duty = 660,newduty;




void main(void){

      //int j ;
      sysctl();
      InitTimer0();
      EPWMInit();

      ADCA_Init();

      DINT;
      IER=00;
      EALLOW;
      PieCtrlRegs.PIECTRL.bit.ENPIE=1;
      PieVectTable.ADCA1_INT= &ADCISR;
     // PieVectTable.EPWM1_INT = &EPWMISR;

 /*  if(a[0] >=16){
      PieVectTable.TIMER0_INT = &Timer0ISR;
      PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
     // PieVectTable.TIMER1_INT

                }*/






      //PieVectTable.EPWM1_INT=&EPWMISR;
      //PieVectTable.EPWM2_INT=&EPWMISR;
      //PieVectTable.EPWM3_INT=&EPWMISR;
      //PieCtrlRegs.PIEACK.bit.ACK3 = 1;
                   // EPwm1Regs.ETCLR.bit.INT = 1;
      PieCtrlRegs.PIEIER1.bit.INTx1=1;
      //ADC A
      //timer 0

      PieCtrlRegs.PIEACK.bit.ACK1=1;
      IER=0x0001;




      EINT;
      EDIS;









          // Update the PWM duty cycle based on the PID output
         // float duty_cycle = constrain(pid_output, 0, 100);
          //setPWMDutyCycle(duty_cycle);

         // delay(dt * 1000);  // Wait for the next iteration*/
}

void EPWMInit(void){
  EALLOW;
  ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV=1;//clk division enable
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
  EPwm1Regs.TBPRD=1000;//set the TB period
  EPwm1Regs.TBCTL.bit.CTRMODE=2;
  EPwm1Regs.CMPA.bit.CMPA = 480;




    //EPwm1Regs.CMPA.bit.CMPA=800;//compare module,compare register


    EPwm2Regs.TBCTL.bit.HSPCLKDIV=2;
    EPwm2Regs.TBPRD=1000;//set the TB period                    17.6 = 960
    EPwm2Regs.TBCTL.bit.CTRMODE=2;//TB mode,counter mode,period 18.2 =              810
    EPwm2Regs.CMPA.bit.CMPA=duty;//re module,compare register // 19.2 volts =  710 = 40
                                                             //  20 volts           = 500 =  50
                                                             //  22.4volts                    = 350 = 60
    EPwm3Regs.TBCTL.bit.HSPCLKDIV=2;                            // 28.8   = 70,210
    EPwm3Regs.TBPRD=1000;//set the TB period       // 42 v = 80 = 120
                                                   //
    EPwm3Regs.TBCTL.bit.CTRMODE=2;//TB mode,counter mode,period
    EPwm3Regs.CMPA.bit.CMPA=200;//compare module,compare register

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



 void ADCA_Init(void)
 {

            EALLOW;

            GpioCtrlRegs.GPAGMUX2.bit.GPIO19 = 01;
            GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 00;
            GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;  //23

            //GpioCtrlRegs.GPAMUX1.bit.GPIO0=01;//configuring gpio pin
             // GpioCtrlRegs.GPAGMUX1.bit.GPIO0=00;
             // GpioCtrlRegs.GPADIR.bit.GPIO0=0;
              // Enable the clock  for the ADC
            CpuSysRegs.PCLKCR13.bit.ADC_A=1;

            // set the ADC  clock frequency
            AdcaRegs.ADCCTL2.bit.PRESCALE=10;

             // select the resolution ,signal  modes
            AdcaRegs.ADCCTL2.bit.RESOLUTION=0;
            AdcaRegs.ADCCTL2.bit.SIGNALMODE=0;

             //VREFHI  and VREFLOW (reference is set based on the voltage available in the launch pad)
            //interrupts pulse position
            AdcaRegs.ADCCTL1.bit.INTPULSEPOS=1;

             //Enable the power to ADC
            AdcaRegs.ADCCTL1.bit.ADCPWDNZ=1;

             // SOC initialization
            AdcaRegs.ADCSOC2CTL.bit.CHSEL=2;      //ADCINA2 = pin 29
            AdcaRegs.ADCSOC2CTL.bit.ACQPS=79;
            AdcaRegs.ADCSOC2CTL.bit.TRIGSEL=01;

            AdcaRegs.ADCSOC3CTL.bit.CHSEL=3;      // ADCINA3 = pin 26
            AdcaRegs.ADCSOC3CTL.bit.ACQPS=79;
            AdcaRegs.ADCSOC3CTL.bit.TRIGSEL=01;

            AdcaRegs.ADCSOC4CTL.bit.CHSEL=4;      //ADCINA4 = 69;


            AdcaRegs.ADCSOC4CTL.bit.ACQPS=79;
            AdcaRegs.ADCSOC4CTL.bit.TRIGSEL=01;

            AdcaRegs.ADCSOC5CTL.bit.CHSEL=5; //ADCINA5 = 66
            AdcaRegs.ADCSOC5CTL.bit.ACQPS=79;
            AdcaRegs.ADCSOC5CTL.bit.TRIGSEL=01;

            // Interrupt pulse position EOC/SOC
            //select the interrupt source
            AdcaRegs.ADCINTSEL1N2.bit.INT1SEL=2;

             //Enable the interrupt
            AdcaRegs.ADCINTSEL1N2.bit.INT1E=1;

             // for continous mode
             //  AdcaRegs.ADCINTSEL1N2.bit.INT1CONT=1;


             //clear the interrupt flag
         //    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;
             EDIS;




 }


              //int i ;






                        // Update the PWM duty cycle based on the PID output
                       // float duty_cycle = constrain(pid_output, 0, 100);
                        //setPWMDutyCycle(duty_cycle);

                       // delay(dt * 1000);  // Wait for the next iteration*/









  void InitTimer0(void)
   {
              EALLOW;
              //   Enable the clock
              CpuSysRegs.PCLKCR0.bit.CPUTIMER0=1;
              //    stop the timer
              CpuTimer0Regs.TCR.bit.TSS=1;
              CpuTimer0Regs.TCR.bit.TIE=0;
              // set the pre scalar register value
              CpuTimer0Regs.TPR.bit.TDDR=32; // set the prescale to system clock frequency to get the output of 16MHZ(
              //  in order to run the timer continously so set it as free run
              CpuTimer0Regs.TCR.bit.FREE=2;
              // set the period register
              //CpuTimer0Regs.PRD.bit.MSW=1;
              CpuTimer0Regs.PRD.all=120;//we can use the period register by using this to access 32 bits
              //disable the interrupt
              CpuTimer0Regs.TCR.bit.TIE=1;
              // load the values to the registers
              CpuTimer0Regs.TCR.bit.TRB=1;
              //clear the flags Timer ,over flow
              CpuTimer0Regs.TCR.bit.TIF=1;
              //start the timer
              CpuTimer0Regs.TCR.bit.TSS=0;
              EDIS;
  }

 /* void timer1(void){
      EALLOW;
      CpuSysRegs.PCLKCR0.bit.CPUTIMER1 = 1;
      CpuTimer1Regs.TCR.bit.TSS =1;
      CpuTimer1Regs.TPR.bit.TDDR = 32;
      CpuTimer1Regs.TCR.bit.FREE = 2;
      CpuTimer1Regs.PRD.all = 120;
      CpuTimer1Regs.TCR.bit.TIE = 1;
      CpuTimer1Regs.TCR.bit.TRB = 1;
      CpuTimer1Regs.TCR.bit.TIF = 1;
      CpuTimer1Regs.TCR.bit.TSS = 0;


  }*/
  __interrupt  void  ADCISR(void)

   {
       EALLOW;
       Digital_Result[0]=AdcaResultRegs.ADCRESULT2;//IN TERMS OF 4096
     //Digital_Result[1]=Digital_Result[0]*(26/4096);
       a[0] = Digital_Result[0]*0.0077000005;

       error = V_ref - Digital_Result[0] ;
       pid_init();


     newduty =  (duty +(int)(pid_output));
           //__interrupt void EPWMISR(void);
   if(newduty < 900 & newduty > 100){

    EPwm2Regs.CMPA.bit.CMPA =  newduty ;}

    //
     //


       /*while(pid1_output >0 )

       //if(pid1_output < 0 )// 8-> 16 -> 370
       {                                          // 9 ->18 ->390
            for(u = 1;u<=25;u++ )
                   v = 0.02 * u;
                    w = 1+v;//010
            { EPwm2Regs.CMPA.bit.CMPA=w*(EPwm2Regs.CMPA.bit.CMPA ) ;//11 -> 23  -> 390
              while(pid1_output!= 0);
              y =  EPwm2Regs.CMPA.bit.CMPA;

            }

                                                                               }*/
                                                                        /* else if(pid1_output == 420){
                                                                                   EPwm1Regs.CMPA.bit.CMPA = 300;
                                                                                  //while(pid_init() != 600.0);

                                                                               }
                                                                               else
                                                                               {
                                                                                   EPwm1Regs.CMPA.bit.CMPA=200;

                                                                               }
                                                                               */





      // V_out =  Digital_Result[1];


   // GpioDataRegs.GPATOGGLE.bit.GPIO31=1;
    //GpioDataRegs.GPATOGGLE.bit.GPIO0=1;
   // CpuTimer0Regs.TCR.bit.TIF = 1;





      // Digital_Result[1]=AdcaResultRegs.ADCRESULT3;//4096*14/4096







       CpuTimer0Regs.TCR.bit.TIF=1;
       //AdcaRegs.ADCINTFLGCLR.bit.ADCINT1=1;
       PieCtrlRegs.PIEIFR1.bit.INTx1= 1;

       PieCtrlRegs.PIEACK.bit.ACK1=1;
  EDIS;
   }

 /*/ __interrupt void Timer0ISR(void)
        {
      EALLOW;
         int k;
         while(1){
                        if( CpuTimer0Regs.TCR.bit.TIF==1) // reading the tif flag
                        { t2 = 0.001;
                          for(k = 1;k<=100;k++){
                          t1 = t2*k ;
                          dt = t1 - t2*(k-1);


                            }
                        }


                              //error = V_ref - a[0] ;
                        //pid_init();

                        /*if(pid1_output< EPwm1Regs.CMPA.bit.CMPA)// 8-> 16 -> 370
                                                                  {                                          // 9 ->18 ->390
                                                                                                             //010
                                                                      EPwm1Regs.CMPA.bit.CMPA = 700;          //11 -> 23  -> 390

                                                                      //while(pid_init()!= 600.0);

                                                                  }
                                                                  else if(pid1_output == 420){
                                                                      EPwm1Regs.CMPA.bit.CMPA = 300;
                                                                     //while(pid_init() != 600.0);

                                                                  }
                                                                  else
                                                                  {
                                                                      EPwm1Regs.CMPA.bit.CMPA=200;

                                                                  }

                                                      //pid1_output = (Kp * error) + (Ki * integral) +(Kd * derivative);
                                                      //pid[0] = pid1_output;

                                                    CpuTimer0Regs.TCR.bit.TIF=1;// when u write 1 to it TIF is cleared.
                                                    PieCtrlRegs.PIEIFR1.bit.INTx7 = 1;
                                                    PieCtrlRegs.PIEACK.bit.ACK1 = 1;










        EDIS;


            }
        }*/


   float pid_init()

           {

                Kp = 0.21;    // 0.05 - 0.07
                Ki = 0.20;    // 0.5
                Kd = 1.7;    // 0.1
                //V_ref = 20.0;  // Desired output voltage   8 - 2.19 v 2280
                //float error ;                                           //10-2.76v 3318

                integral = 0.0;                            //  12 -3.12v 3800
                previous_error = 0.0;
                derivative = 0;
                dt = 1;  // Time interval in seconds //0.01

                // Read output voltage
                // error = V_ref - a[0] ;

                integral += error ;
                derivative = (error - previous_error)/dt;
                pid1_output = (Kp * error) +(Ki * integral); +(Kd * derivative);
                previous_error = error;
                pid_output = pid1_output;
                return  pid_output;

           // pid out_put = 0.8297; 0.8 volts
  }
   /*__interrupt void EPWMISR(void)
          {    EPwm2Regs.CMPA.bit.CMPA = EPwm2Regs.CMPA.bit.CMPA - pid1_output;
              PieCtrlRegs.PIEACK.bit.ACK3 = 1;
              EPwm1Regs.ETCLR.bit.INT = 1;
          }*
