#ifndef MODULATION_H
#define MODULATION_H

typedef enum
{
  PingPong1,
  PingPong2,
  PingPong3,
  PingPong4
} ModulationState;

typedef enum
{
  VoltagePeak,
  IsensePhase
} ModulationType;

extern ModulationState modulation_state;                /*!< Current type of modulation (PingPong) being used */
extern ModulationType modulation_type;                  /*!< Type of modulation being used (V or I) */
extern int hysteresis_scale_factor;                     /*!< Shift factor used for hysteresis calculation */

extern void initModulation(void);                       /*!< Initialize modulation */
extern void resetModulation(void);                      /*!< Resets modulation */
extern int resetHysteresis(void);                       /*!< Resets dynamic hysteresis accumulation to default value for this modulation type */
extern void resetDynamicHysteresis(void);               /*!< Resets dynamic hysteresis to 0 */
extern void advanceModulation(void);                    /*!< Advance to next modulation state and return new state */  
extern int limitLpFilterHys(int raw_hys);  

#endif // MODULATION_H

