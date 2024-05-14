/* Modelled after the way MSPGCC handles the inclusion of model specific headers */
#ifndef _IO_H_
#define _IO_H_

#if defined(__IAR_SYSTEMS_ICC__) || defined(__IAR_SYSTEMS_ASM__)  ||  defined(__TI_COMPILER_VERSION__)

/* Translate the defines generated by IAR */

/********************************************************************
 *  cc430xxxx family 
 ********************************************************************/

#if defined(__CC430F5133__)
#define __CC430_5133__

#elif defined(__CC430F5135__)
#define __CC430_5135__

#elif defined(__CC430F5137__)
#define __CC430_5137__

/********************************************************************
 *  msp430x1xx family 
 ********************************************************************/
#elif defined(__MSP430C111__) || defined(__MSP430F111__)
#define __MSP430_111__

#elif defined(__MSP430C112__) || defined(__MSP430E112__) || defined(__MSP430F112__)
#define __MSP430_112__

#elif defined(__MSP430F1101__) || defined(__MSP430F1101A__)
#define __MSP430_1101__

#elif defined(__MSP430C1111__) || defined(__MSP430F1111__) || defined(__MSP430F1111A__)
#define __MSP430_1111__

#elif defined(__MSP430C1121__) || defined(__MSP430F1121__) || defined(__MSP430F1121A__)
#define __MSP430_1121__

#elif defined(__MSP430F1122__)
#define __MSP430_1122__

#elif defined(__MSP430F1132__)
#define __MSP430_1132__

#elif defined(__MSP430F1222__)
#define __MSP430_1222__

#elif defined(__MSP430F1232__)
#define __MSP430_1232__

#elif defined(__MSP430C1331__) || defined(__MSP430F1331__)
#define __MSP430_1331__

#elif defined(__MSP430C1351__) || defined(__MSP430F1351__)
#define __MSP430_1351__

#elif defined(__MSP430F133__) || defined(__MSP430F135__)
#define __MSP430_133__

#elif defined(__MSP430F1471__) || defined(__MSP430F1481__) || defined(__MSP430F1491__)
#define __MSP430_1471__

#elif defined(__MSP430F147__) || defined(__MSP430F148__) || defined(__MSP430F149__)
#define __MSP430_147__

#elif defined(__MSP430F155__) || defined(__MSP430F156__) || defined(__MSP430F157__)
#define __MSP430_155__

#elif defined(__MSP430F167__)
#define __MSP430_167__

#elif defined(__MSP430F168__)
#define __MSP430_168__

#elif defined(__MSP430F169__)
#define __MSP430_169__

#elif defined(__MSP430F1610__)
#define __MSP430_1610__

#elif defined(__MSP430F1611__)
#define __MSP430_1611__

#elif defined(__MSP430F1612__)
#define __MSP430_1612__


/********************************************************************
 *  msp430x2xx family 
 ********************************************************************/
#elif defined(__MSP430F2001__)
#define __MSP430_2001__

#elif defined(__MSP430F2011__)
#define __MSP430_2011__

#elif defined(__MSP430F2002__)
#define __MSP430_2002__

#elif defined(__MSP430F2012__)
#define __MSP430_2012__

#elif defined(__MSP430F2012__)
#define __MSP430_2012__

#elif defined(__MSP430F2003__)
#define __MSP430_2003__

#elif defined(__MSP430F2013__)
#define __MSP430_2013__

#elif defined(__MSP430F2101__)
#define __MSP430_2101__

#elif defined(__MSP430F2111__)
#define __MSP430_2111__

#elif defined(__MSP430F2121__)
#define __MSP430_2121__

#elif defined(__MSP430F2131__)
#define __MSP430_2131__

#elif defined(__MSP430F2232__)
#define __MSP430_2232__

#elif defined(__MSP430F2252__)
#define __MSP430_2252__

#elif defined(__MSP430F2272__)
#define __MSP430_2272__

#elif defined(__MSP430F2234__)
#define __MSP430_2234__

#elif defined(__MSP430F2354__)
#define __MSP430_2254__

#elif defined(__MSP430F2274__)
#define __MSP430_2274__

#elif defined(__MSP430F2330__)
#define __MSP430_2330__

#elif defined(__MSP430F2350__)
#define __MSP430_2350__

#elif defined(__MSP430F2370__)
#define __MSP430_2370__


/********************************************************************
 *  msp430x3xx family 
 ********************************************************************/
#elif defined(__MSP430C311S__)
#define __MSP430_311__

#elif defined(__MSP430C312__)
#define __MSP430_312__

#elif defined(__MSP430C313__) || defined(__MSP430EC313__) || defined(__MSP430P313__)
#define __MSP430_313__

#elif defined(__MSP430C314__)
#define __MSP430_314__

#elif defined(__MSP430C315__) || defined(__MSP430E315__) || defined(__MSP430P315__)  || defined(__MSP430P315S__)
#define __MSP430_315__

#elif defined(__MSP430C323__) || defined(__MSP430C325__)
#define __MSP430_323__

#elif defined(__MSP430E325__) || defined(__MSP430P325__)
#define __MSP430_325__

#elif defined(__MSP430C336__)
#define __MSP430_336__

#elif defined(__MSP430C337__) || defined(__MSP430E337__) || defined(__MSP430P337__)
#define __MSP430_337__


/********************************************************************
 *  msp430x4xx family 
 ********************************************************************/
#elif defined(__MSP430i2040__)

#elif defined(__MSP430i2041__)

#elif defined(__MSP430i4020__)

#elif defined(__MSP430C412__) || defined(__MSP430F412__)
#define __MSP430_412__

#elif defined(__MSP430C413__) || defined(__MSP430F413__)
#define __MSP430_413__

#elif defined(__MSP430F415__) 
#define __MSP430_415__

#elif defined(__MSP430F417__) 
#define __MSP430_417__

#elif defined(__MSP430F4250__)
#define __MSP430_4250__

#elif defined(__MSP430F4260__)
#define __MSP430_4260__

#elif defined(__MSP430F4270__)
#define __MSP430_4270__

#elif defined(__MSP430F423__)
#define __MSP430_423__

#elif defined(__MSP430F425__)
#define __MSP430_425__

#elif defined(__MSP430F427__)
#define __MSP430_427__

#elif defined(__MSP430F423A__)
#define __MSP430_423A__

#elif defined(__MSP430F425A__)
#define __MSP430_425A__

#elif defined(__MSP430F427A__)
#define __MSP430_427A__

#elif defined(__MSP430FE423__)
#define __MSP430_E423__

#elif defined(__MSP430FE425__)
#define __MSP430_E425__

#elif defined(__MSP430FE427__)
#define __MSP430_E427__

#elif defined(__MSP430FE423A__)
#define __MSP430_E423A__

#elif defined(__MSP430FE425A__)
#define __MSP430_E425A__

#elif defined(__MSP430FE427A__)
#define __MSP430_E427A__

#elif defined(__MSP430FE4232__)
#define __MSP430_E4232__

#elif defined(__MSP430FE4242__)
#define __MSP430_E4242__

#elif defined(__MSP430FE4252__)
#define __MSP430_E4252__

#elif defined(__MSP430FE4272__)
#define __MSP430_E4272__

#elif defined(__MSP430W423__)
#define __MSP430_W423__

#elif defined(__MSP430W425__)
#define __MSP430_W425__

#elif defined(__MSP430W427__)
#define __MSP430_W427__

#elif defined(__MSP430F435__)
#define __MSP430_435__

#elif defined(__MSP430F436__)
#define __MSP430_436__

#elif defined(__MSP430F437__)
#define __MSP430_437__

#elif defined(__MSP430FG437__)
#define __MSP430_G437__

#elif defined(__MSP430FG438__)
#define __MSP430_G438__

#elif defined(__MSP430FG439__)
#define __MSP430_G439__

#elif defined(__MSP430F447__)
#define __MSP430_447__

#elif defined(__MSP430F448__)
#define __MSP430_448__

#elif defined(__MSP430F449__)
#define __MSP430_449__

#elif defined(__MSP430FG4616__)
#define __MSP430_G4616__

#elif defined(__MSP430FG4617__)
#define __MSP430_G4617__

#elif defined(__MSP430FG4618__)
#define __MSP430_G4618__

#elif defined(__MSP430FG4619__)
#define __MSP430_G4619__

#elif defined(__MSP430F4783__)
#define __MSP430_4783__

#elif defined(__MSP430F4793__)
#define __MSP430_4793__

#elif defined(__MSP430F4784__)
#define __MSP430_4784__

#elif defined(__MSP430F4794__)
#define __MSP430_4794__

#elif defined(__MSP430F47166__)
#define __MSP430_47166__

#elif defined(__MSP430F47176__)
#define __MSP430_47176__

#elif defined(__MSP430F47186__)
#define __MSP430_47186__

#elif defined(__MSP430F47196__)
#define __MSP430_47196__

#elif defined(__MSP430F47167__)
#define __MSP430_47167__

#elif defined(__MSP430F47177__)
#define __MSP430_47177__

#elif defined(__MSP430F47187__)
#define __MSP430_47187__

#elif defined(__MSP430F47197__)
#define __MSP430_47197__

#elif defined(__MSP430F6700__)
#define __MSP430_6700__

#elif defined(__MSP430F6701__)
#define __MSP430_6701__

#elif defined(__MSP430F6702__)
#define __MSP430_6702__

#elif defined(__MSP430F6703__)
#define __MSP430_6703__

#elif defined(__MSP430F6720__)
#define __MSP430_6720__

#elif defined(__MSP430F6721__)
#define __MSP430_6721__

#elif defined(__MSP430F6722__)
#define __MSP430_6722__

#elif defined(__MSP430F6723__)
#define __MSP430_6723__

#elif defined(__MSP430F6730__)
#define __MSP430_6730__

#elif defined(__MSP430F6731__)
#define __MSP430_6731__

#elif defined(__MSP430F6732__)
#define __MSP430_6732__

#elif defined(__MSP430F6733__)
#define __MSP430_6733__

#elif defined(__MSP430F6736__)
#define __MSP430_6736__

#elif defined(__MSP430F67641__)
#define __MSP430_67641__

#elif defined(__MSP430F6779__)
#define __MSP430_6779__

#elif defined(__MSP430AFE221__)  ||  defined(__MSP430AFE222__) ||  defined(__MSP430AFE223__)
#elif defined(__MSP430AFE231__)  ||  defined(__MSP430AFE232__) ||  defined(__MSP430AFE233__)
#elif defined(__MSP430AFE251__)  ||  defined(__MSP430AFE252__) ||  defined(__MSP430AFE253__)

#else
#error "Failed to match a default include file"
#endif

#endif

#if defined(__CC430F5133__)
#include <cc430f5133.h>

#elif defined(__CC430F5135__)
#include <cc430f5135.h>

#elif defined(__CC430F5137__)
#include <cc430f5137.h>

#elif defined(__MSP430_1101__) || defined(__MSP430_1111__) || defined(__MSP430_1121__)
#include <msp430x11x1.h>

#elif defined(__MSP430_110__) || defined(__MSP430_112__)
#include <msp430x11x.h>

#elif defined(__MSP430_122__) || defined(__MSP430_123__)
#include <msp430x12x.h>

#elif defined(__MSP430_1122__) || defined(__MSP430_1132__)
#include <msp430x11x2.h>

#elif defined(__MSP430_1222__) || defined(__MSP430_1232__)
#include <msp430x12x2.h>

#elif defined(__MSP430_133__) || defined(__MSP430_135__)
#include <msp430x13x.h>

#elif defined(__MSP430_147__) || defined(__MSP430_148__) || defined(__MSP430_149__)
#include <msp430x14x.h>

#elif defined(__MSP430_1331__) || defined(__MSP430_1351__)
#include <msp430x13x1.h>

#elif defined(__MSP430_1471__) || defined(__MSP430_1481__) || defined(__MSP430_1491__)
#include <msp430x14x1.h>

#elif defined(__MSP430_155__) || defined(__MSP430_156__) || defined(__MSP430_157__)
#include <msp430x15x.h>

#elif defined(__MSP430_167__) || defined(__MSP430_168__) || defined(__MSP430_169__) || defined(__MSP430_1610__) || defined(__MSP430_1611__) || defined(__MSP430_1612__)
#include <msp430x16x.h>

#elif defined(__MSP430_2001__) || defined(__MSP430_2011__)
#include <msp430x20x1.h>

#elif defined(__MSP430_2002__) || defined(__MSP430_2012__)
#include <msp430x20x2.h>

#elif defined(__MSP430_2003__) || defined(__MSP430_2013__)
#include <msp430x20x3.h>

#elif defined(__MSP430_2101__) || defined(__MSP430_2111__) || defined(__MSP430_2121__) || defined(__MSP430_2131__)
#include <msp430x21x1.h>

#elif defined(__MSP430_2234__) || defined(__MSP430_2254__) || defined(__MSP430_2274__)
#include <msp430x22x4.h>

#elif defined(__MSP430AFE221__)
#include <msp430afe221.h>

#elif defined(__MSP430AFE222__)
#include <msp430afe222.h>

#elif defined(__MSP430AFE223__)
#include <msp430afe223.h>

#elif defined(__MSP430AFE231__)
#include <msp430afe231.h>

#elif defined(__MSP430AFE232__)
#include <msp430afe232.h>

#elif defined(__MSP430AFE233__)
#include <msp430afe233.h>

#elif defined(__MSP430AFE251__)
#include <msp430afe251.h>

#elif defined(__MSP430AFE252__)
#include <msp430afe252.h>

#elif defined(__MSP430AFE253__)
#include <msp430afe253.h>

#elif defined(__MSP430_311__) || defined(__MSP430_312__) || defined(__MSP430_313__) || defined(__MSP430_314__) || defined(__MSP430_315__)
#include <msp430x31x.h>

#elif defined(__MSP430_323__) || defined(__MSP430_325__)
#include <msp430x32x.h>

#elif defined(__MSP430_336__) || defined(__MSP430_337__)
#include <msp430x33x.h>

#elif defined(__MSP430i2040__)
#include <msp430i2040.h>

#elif defined(__MSP430i2041__)
#include <msp430i2041.h>

#elif defined(__MSP430i4020__)
#include <msp430i4020.h>

#elif defined(__MSP430_412__) || defined(__MSP430_413__) || defined(__MSP430_415__) || defined(__MSP430_417__)
#include <msp430x41x.h>

#elif defined(__MSP430_423__) || defined(__MSP430_425__) || defined(__MSP430_427__)
#include <msp430x42x.h>

#elif defined(__MSP430_423A__) || defined(__MSP430_425A__) || defined(__MSP430_427A__)
#include <msp430x42x.h>

#elif defined(__MSP430_4250__) || defined(__MSP430_4260__) || defined(__MSP430_4270__)
#include <msp430x42x0.h>

#elif defined(__MSP430_E423__) || defined(__MSP430_E425__) || defined(__MSP430_E427__)
#include <msp430xE42x.h>

#elif defined(__MSP430_E423A__) || defined(__MSP430_E425A__) || defined(__MSP430_E427A__)
#include <msp430xE42x.h>

#elif defined(__MSP430_E4232__) || defined(__MSP430_E4242__) || defined(__MSP430_E4252__) || defined(__MSP430_E4272__)
#include <msp430xE42x2.h>

#elif defined(__MSP430_W423__) || defined(__MSP430_W425__) || defined(__MSP430_W427__)
#include <msp430xW42x.h>

#elif defined(__MSP430_G437__) || defined(__MSP430_G438__) || defined(__MSP430_G439__)
#include <msp430xG43x.h>

#elif defined(__MSP430_435__) || defined(__MSP430_436__) || defined(__MSP430_437__)
#include <msp430x43x.h>

#elif defined(__MSP430_447__) || defined(__MSP430_448__) || defined(__MSP430_449__)
#include <msp430x44x.h>

#elif defined(__MSP430_G4616__) || defined(__MSP430_G4617__) || defined(__MSP430_G4618__) || defined(__MSP430_G4619__)
#include <msp430xG46x.h>

#elif defined(__MSP430_4783__) || defined(__MSP430_4793__)
#include <msp430x47x3.h>

#elif defined(__MSP430_4784__) || defined(__MSP430_4794__)
#include <msp430x47x4.h>

#elif defined(__MSP430_47166__) || defined(__MSP430_47176__) || defined(__MSP430_47186__) || defined(__MSP430_47196__)
#include <msp430x471x6.h>

#elif defined(__MSP430_47167__) || defined(__MSP430_47177__) || defined(__MSP430_47187__) || defined(__MSP430_47197__)
#include <msp430x471x7.h>

#elif defined(__MSP430_6700__)
#include <msp430f6700.h>

#elif defined(__MSP430_6701__)
#include <msp430f6701.h>

#elif defined(__MSP430_6702__)
#include <msp430f6702.h>

#elif defined(__MSP430_6703__)
#include <msp430f6703.h>

#elif defined(__MSP430_6720__)
#include <msp430f6720.h>

#elif defined(__MSP430_6721__)
#include <msp430f6721.h>

#elif defined(__MSP430_6722__)
#include <msp430f6722.h>

#elif defined(__MSP430_6723__)
#include <msp430f6723.h>

#elif defined(__MSP430_6730__)
#include <msp430f6730.h>

#elif defined(__MSP430_6731__)
#include <msp430f6731.h>

#elif defined(__MSP430_6732__)
#include <msp430f6732.h>

#elif defined(__MSP430_6733__)
#include <msp430f6733.h>

#elif defined(__MSP430_6736__)
#include <msp430f6736.h>

#elif defined(__MSP430_67641__)
#include <msp430f67641.h>

#elif defined(__MSP430_6779__)
#include <msp430f6779.h>

#elif defined(__MSP430__)
#error "Unknown architecture! Please check"
#include <iomacros.h>
#endif

#endif
