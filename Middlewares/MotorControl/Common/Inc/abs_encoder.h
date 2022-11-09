#ifndef _ABS_ENCODER_H_
#define _ABS_ENCODER_H_

/* Includes ------------------------------------------------------------------*/
#include "mc_type.h"
#include "speed_pos_fdbk.h"

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

/**
 * @brief ABS_ENCODER component parameters definition
 *
 *  <Type @p type represents a thing that needs to be detailed more. Additional
 * details are provided in the detailed section of the doxygen comment block.
 *
 * The brief line should be brief and light. It should avoid useless repetitions
 * and expression such as "the CCC_Type_t type...". Expressions like "This type
 * ..." are tolerated especially for key types (usually structures) where we may
 * want ot be more formal.
 *
 * In general: be direct, avoid the obvious, tell the hidden.>
 */
typedef struct abs_encoder {
    speedn_pos_fdbk_t _Super;

    /* hw Settings */
    bool sensor_reliable; /*!< Flag to indicate a wrong configuration
                           of the Hall sensor signanls.*/

    int16_t current_speed; /*!< Latest speed computed in HALL_IRQ_HANDLER*/

    int8_t direction; /*!< Instantaneous direction of rotor between two
                       captures*/

    int16_t avr_el_speed_dpp; /*!< It is the averaged rotor electrical speed express
                               in s16degree per current control period.*/

    int16_t delta_angle;       /*!< Delta angle at the Hall sensor signal edge between
                                current electrical rotor angle of synchronism.
                                It is in s16degrees.*/
    int16_t measured_el_angle; /*!< This is the electrical angle  measured at
                                each Hall sensor signal edge. It is considered
                                the best measurement of electrical rotor angle.*/

    int16_t target_el_angle; /*!< This is the electrical angle target computed at
                              speed control frequency based on hmeasured_el_angle.*/

    int32_t circle_counter; /* the number of turns */

    uint16_t mec_angle; /* Mechanics Rotor position */

    int16_t mec_to_dpp; /* Mechanics angle to MecAngleDpp scale */

    uint16_t mec_angle_dpp; /* Mechanics Rotor position with dpp */

    uint16_t last_mec_angle_dpp; /* Last Mechanics Rotor position with dpp*/

    int16_t el_angle_dpp; /* Electronics Rotor position with dpp */

    int16_t mec_offset_to_el_dpp; /* Electronics offset base on Mechanics angle */

    int32_t mec_pos; /* Mechanics positions in speed loop */

    int32_t last_mec_pos; /* Mechanics positions in speed loop */

    uint32_t speed_ticks; /* current ticks in speed loop */

    uint32_t last_speed_ticks; /* last ticks in speed loop */

    void (*abs_encoder_device_init)(void);
    uint16_t (*abs_encoder_read_angle)(void);
    uint16_t (*abs_encoder_read_speed)(void);

} abs_encoder_t;

void abs_encoder_init(abs_encoder_t* phandle);
int16_t abs_encoder_calc_el_angle(abs_encoder_t* phandle);
bool abs_encoder_reliable(abs_encoder_t* phandle);

#ifdef __cplusplus
}
#endif /* __cpluplus */

#endif
