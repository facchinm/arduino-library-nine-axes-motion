#ifndef __BNO055_HPP__
#define __BNO055_HPP__

#include "Arduino.h"
#include "utility/BN055.h"

class BN055Class {
    public:
		STD_RET Init(struct bno055T *bno055){ return bno055_init(bno055); } 
		STD_RET WriteRegister(u8 vAddrU8, u8 *pDataU8, u8 vLenU8){ bno055_write_register(p_data_u_8, v_len_u_8); } 
		STD_RET ReadRegister(u8 vAddrU8, u8 *pDataU8, u8 vLenU8){ bno055_read_register(p_data_u_8, v_len_u_8); } 
		STD_RET ReadChipId(u8 *vChipIdU8){ return bno055_read_chip_id(*v_chip_id_u_8); } 
		STD_RET ReadSwRevId(u16 *vSwIdU8){ return bno055_read_sw_rev_id(v_sw_id_u_8); } 
		STD_RET ReadPageId(u8 *vPageIdU8){ return bno055_read_page_id(v_page_id_u_8); } 
		STD_RET WritePageId(u8 vPageIdU8){ return bno055_write_page_id(v_page_id_u_8); } 
		STD_RET ReadAccelRevId(u8 *vAccelRevIdU8){ return bno055_read_accel_rev_id(v_accel_rev_id_u_8); } 
		STD_RET ReadMagRevId(u8 *vMagRevIdU8){ return bno055_read_mag_rev_id(v_mag_rev_id_u_8); } 
		STD_RET ReadGyroRevId(u8 *vGyroRevIdU8){ return bno055_read_gyro_rev_id(v_gyro_rev_id_u_8); } 
		STD_RET ReadBlRevId(u8 *vBlRevIdU8){ return bno055_read_bl_rev_id(v_bl_rev_id_u_8); } 
		STD_RET ReadAccelX(s16 *vAccelXS16){ return bno055_read_accel_x(v_accel_x_s_16); } 
		STD_RET ReadAccelY(s16 *vAccelYS16){ return bno055_read_accel_y(v_accel_y_s_16); } 
		STD_RET ReadAccelZ(s16 *vAccelZS16){ return bno055_read_accel_z(v_accel_z_s_16); } 
		STD_RET ReadAccelXyz(struct bno055AccelT *accel){ return bno055_read_accel_xyz(accel); } 
		STD_RET ReadMagX(s16 *vMagXS16){ return bno055_read_mag_x(v_mag_x_s_16); } 
		STD_RET ReadMagY(s16 *vMagYS16){ return bno055_read_mag_y(v_mag_y_s_16); } 
		STD_RET ReadMagZ(s16 *vMagZS16){ return bno055_read_mag_z(v_mag_z_s_16); } 
		STD_RET ReadMagXyz(struct bno055MagT *mag){ return bno055_read_mag_xyz(mag); } 

		STD_RET ReadGyroX(s16 *vGyroXS16){ return bno055_read_gyro_x(v_gyro_x_s_16); } 
		STD_RET ReadGyroY(s16 *vGyroYS16){ return bno055_read_gyro_y(v_gyro_y_s_16); } 
		STD_RET ReadGyroZ(s16 *vGyroZS16){ return bno055_read_gyro_z(v_gyro_z_s_16); } 
		STD_RET ReadGyroXyz(struct bno055GyroT *gyro){ return bno055_read_gyro_xyz(gyro); } 

		STD_RET ReadEulerH(s16 *vEulerHS16){ return bno055_read_euler_h(v_euler_h_s_16); } 
		STD_RET ReadEulerR(s16 *vEulerRS16){ return bno055_read_euler_r(v_euler_r_s_16); } 
		STD_RET ReadEulerP(s16 *vEulerPS16){ return bno055_read_euler_p(v_euler_p_s_16); } 
		STD_RET ReadEulerHrp(struct bno055EulerT *euler){ return bno055_read_euler_hrp(euler); } 
		STD_RET ReadQuaternionW(s16 *vQuaternionWS16){ return bno055_read_quaternion_w(v_quaternion_w_s_16); } 
		STD_RET ReadQuaternionX(s16 *vQuaternionXS16){ return bno055_read_quaternion_x(v_quaternion_x_s_16); } 
		STD_RET ReadQuaternionY(s16 *vQuaternionYS16){ return bno055_read_quaternion_y(v_quaternion_y_s_16); } 
		STD_RET ReadQuaternionZ(s16 *vQuaternionZS16){ return bno055_read_quaternion_z(v_quaternion_z_s_16); } 
		STD_RET ReadQuaternionWxyz(struct bno055QuaternionT *quaternion){ return bno055_read_quaternion_wxyz(quaternion); } 
		STD_RET ReadLinearAccelX(s16 *vLinearAccelXS16){ return bno055_read_linear_accel_x(v_linear_accel_x_s_16); } 
		STD_RET ReadLinearAccelY(s16 *vLinearAccelYS16){ return bno055_read_linear_accel_y(v_linear_accel_y_s_16); } 
		STD_RET ReadLinearAccelZ(s16 *vLinearAccelZS16){ return bno055_read_linear_accel_z(v_linear_accel_z_s_16); } 
		STD_RET ReadLinearAccelXyz(struct bno055LinearAccelT *linearAccel){ return bno055_read_linear_accel_xyz(linear_accel); } 
		STD_RET ReadGravityX(s16 *vGravityXS16){ return bno055_read_gravity_x(v_gravity_x_s_16); }
		STD_RET ReadGravityY(s16 *vGravityYS16){ return bno055_read_gravity_y(v_gravity_y_s_16); } 
		STD_RET ReadGravityZ(s16 *vGravityZS16){ return bno055_read_gravity_z(v_gravity_z_s_16); } 
		STD_RET ReadGravityXyz(struct bno055GravityT *gravity){ return bno055_read_gravity_xyz(gravity); } 
		STD_RET ReadTempData(s8 *vTempS8){ return bno055_read_temp_data(v_temp_s_8); } 
		STD_RET ConvertFloatAccelXMsq(float *vAccelXF){ return bno055_convert_float_accel_x_msq(v_accel_x_f); } 
		STD_RET ConvertFloatAccelYMsq(float *vAccelYF){ return bno055_convert_float_accel_y_msq(v_accel_y_f); } 
		STD_RET ConvertFloatAccelZMsq(float *vAccelZF){ return bno055_convert_float_accel_z_msq(v_accel_z_f); } 
		STD_RET ConvertFloatAccelXMg(float *vAccelXF){ return bno055_convert_float_accel_x_mg(v_accel_x_f); } 
		STD_RET ConvertFloatAccelYMg(float *vAccelYF){ return bno055_convert_float_accel_y_mg(v_accel_y_f); } 
		STD_RET ConvertFloatAccelZMg(float *vAccelZF){ return bno055_convert_float_accel_z_mg(v_accel_z_f); } 
		STD_RET ConvertFloatAccelXyzMsq(struct bno055AccelFloatT *accelXyz){ return bno055_convert_float_accel_xyz_msq(accel_xyz); } 
		STD_RET ConvertFloatAccelXyzMg(struct bno055AccelFloatT *accelXyz){ return bno055_convert_float_accel_xyz_mg(accel_xyz); } 
		STD_RET ConvertFloatMagXUT(float *vMagXF){ return bno055_convert_float_mag_x_uT(v_mag_x_f); } 
		STD_RET ConvertFloatMagYUT(float *vMagYF){ return bno055_convert_float_mag_y_uT(v_mag_y_f); } 
		STD_RET ConvertFloatMagZUT(float *vMagZF){ return bno055_convert_float_mag_z_uT(v_mag_z_f); } 
		STD_RET ConvertFloatMagXyzUT(struct bno055MagFloatT *magXyzData){ return bno055_convert_float_mag_xyz_uT(mag_xyz_data); } 
		STD_RET ConvertFloatGyroXDps(float *vGyroXF){ return bno055_convert_float_gyro_x_dps(v_gyro_x_f); } 
		STD_RET ConvertFloatGyroXRps(float *vGyroXF){ return bno055_convert_float_gyro_x_rps(v_gyro_x_f); } 
		STD_RET ConvertFloatGyroYDps(float *vGyroYF){ return bno055_convert_float_gyro_y_dps(v_gyro_y_f); } 
		STD_RET ConvertFloatGyroYRps(float *vGyroYF){ return bno055_convert_float_gyro_y_rps(v_gyro_y_f); } 
		STD_RET ConvertFloatGyroZDps(float *vGyroZF){ return bno055_convert_float_gyro_z_dps(v_gyro_z_f); } 
		STD_RET ConvertFloatGyroZRps(float *vGyroZF){ return bno055_convert_float_gyro_z_rps(v_gyro_z_f); } 
		STD_RET ConvertFloatGyroXyzDps(struct bno055GyroFloatT *gyroXyzData){ return bno055_convert_float_gyro_xyz_dps(gyro_xyz_data); } 
		STD_RET ConvertFloatGyroXyzRps(struct bno055GyroFloatT *gyroXyzData){ return bno055_convert_float_gyro_xyz_rps(gyro_xyz_data); } 
		STD_RET ConvertFloatEulerHDeg(float *vEulerHF){ return bno055_convert_float_euler_h_deg(v_euler_h_f); } 
		STD_RET ConvertFloatEulerHRad(float *vEulerHF){ return bno055_convert_float_euler_h_rad(v_euler_h_f); } 
		STD_RET ConvertFloatEulerRDeg(float *vEulerRF){ return bno055_convert_float_euler_r_deg(v_euler_r_f); } 
		STD_RET ConvertFloatEulerRRad(float *vEulerRF){ return bno055_convert_float_euler_r_rad(v_euler_r_f); } 
		STD_RET ConvertFloatEulerPDeg(float *vEulerPF){ return bno055_convert_float_euler_p_deg(v_euler_p_f); } 
		STD_RET ConvertFloatEulerPRad(float *vEulerPF){ return bno055_convert_float_euler_p_rad(v_euler_p_f); } 
		STD_RET ConvertFloatEulerHprDeg(struct bno055EulerFloatT *eulerHpr){ return bno055_convert_float_euler_hpr_deg(euler_hpr); } 
		STD_RET ConvertFloatEulerHprRad(struct bno055EulerFloatT *eulerHpr){ return bno055_convert_float_euler_hpr_rad(euler_hpr); } 
		STD_RET ConvertFloatLinearAccelXMsq(float *vLinearAccelXF){ return bno055_convert_float_linear_accel_x_msq(v_linear_accel_x_f); } 
		STD_RET ConvertFloatLinearAccelYMsq(float *vLinearAccelYF){ return bno055_convert_float_linear_accel_y_msq(v_linear_accel_y_f); } 
		STD_RET ConvertFloatLinearAccelZMsq(float *vLinearAccelZF){ return bno055_convert_float_linear_accel_z_msq(v_linear_accel_z_f); } 
		STD_RET ConvertFloatLinearAccelXyzMsq(struct bno055LinearAccelFloatT *linearAccelXyz){ return bno055_convert_float_linear_accel_xyz_msq(linear_accel_xyz); } 
		STD_RET ConvertGravityFloatXMsq(float *vGravityXF){ return bno055_convert_gravity_float_x_msq(v_gravity_x_f); } 
		STD_RET ConvertGravityFloatYMsq(float *vGravityYF){ return bno055_convert_gravity_float_y_msq(v_gravity_y_f); } 
		STD_RET ConvertGravityFloatZMsq(float *gravityZ){ return bno055_convert_gravity_float_z_msq(gravity_z); } 
		STD_RET ConvertFloatGravityXyzMsq(struct bno055GravityFloatT *gravityXyz){ return bno055_convert_float_gravity_xyz_msq(gravity_xyz); } 
		STD_RET ConvertFloatTempFahrenheit(float *vTempF){ return bno055_convert_float_temp_fahrenheit(v_temp_f); } 
		STD_RET ConvertFloatTempCelsius(float *vTempF){ return bno055_convert_float_temp_celsius(v_temp_f); } 
		STD_RET ConvertDoubleAccelXMsq(double *vAccelXD){ return bno055_convert_double_accel_x_msq(v_accel_x_d); } 
		STD_RET ConvertDoubleAccelYMsq(double *vAccelYD){ return bno055_convert_double_accel_y_msq(v_accel_y_d); } 
		STD_RET ConvertDoubleAccelZMsq(double *vAccelZD){ return bno055_convert_double_accel_z_msq(v_accel_z_d); } 
		STD_RET ConvertDoubleAccelXMg(double *vAccelXD){ return bno055_convert_double_accel_x_mg(v_accel_x_d); } 
		STD_RET ConvertDoubleAccelYMg(double *vAccelYD){ return bno055_convert_double_accel_y_mg(v_accel_y_d); } 
		STD_RET ConvertDoubleAccelZMg(double *vAccelZD){ return bno055_convert_double_accel_z_mg(v_accel_z_d); } 
		STD_RET ConvertDoubleAccelXyzMsq(struct bno055AccelDoubleT *accelXyz){ return bno055_convert_double_accel_xyz_msq(accel_xyz); } 
		STD_RET ConvertDoubleAccelXyzMg(struct bno055AccelDoubleT *accelXyz){ return bno055_convert_double_accel_xyz_mg(accel_xyz); } 
		STD_RET ConvertDoubleMagXUT(double *vMagXD){ return bno055_convert_double_mag_x_uT(v_mag_x_d); } 
		STD_RET ConvertDoubleMagYUT(double *vMagYD){ return bno055_convert_double_mag_y_uT(v_mag_y_d); } 
		STD_RET ConvertDoubleMagZUT(double *vMagZD){ return bno055_convert_double_mag_z_uT(v_mag_z_d); } 
		STD_RET ConvertDoubleMagXyzUT(struct bno055MagDoubleT *magXyz){ return bno055_convert_double_mag_xyz_uT(mag_xyz); } 
		STD_RET ConvertDoubleGyroXDps(double *vGyroXD){ return bno055_convert_double_gyro_x_dps(v_gyro_x_d); } 
		STD_RET ConvertDoubleGyroYDps(double *vGyroYD){ return bno055_convert_double_gyro_y_dps(v_gyro_y_d); } 
		STD_RET ConvertDoubleGyroZDps(double *vGyroZD){ return bno055_convert_double_gyro_z_dps(v_gyro_z_d); } 
		STD_RET ConvertDoubleGyroXRps(double *vGyroXD){ return bno055_convert_double_gyro_x_rps(v_gyro_x_d); } 
		STD_RET ConvertDoubleGyroYRps(double *vGyroYD){ return bno055_convert_double_gyro_y_rps(v_gyro_y_d); } 
		STD_RET ConvertDoubleGyroZRps(double *vGyroZD){ return bno055_convert_double_gyro_z_rps(v_gyro_z_d); } 
		STD_RET ConvertDoubleGyroXyzDps(struct bno055GyroDoubleT *gyroXyz){ return bno055_convert_double_gyro_xyz_dps(gyro_xyz); } 
		STD_RET ConvertDoubleGyroXyzRps(struct bno055GyroDoubleT *gyroXyz){ return bno055_convert_double_gyro_xyz_rps(gyro_xyz); } 
		STD_RET ConvertDoubleEulerHDeg(double *vEulerHD){ return bno055_convert_double_euler_h_deg(v_euler_h_d); } 
		STD_RET ConvertDoubleEulerPDeg(double *vEulerPD){ return bno055_convert_double_euler_p_deg(v_euler_p_d); } 
		STD_RET ConvertDoubleEulerRDeg(double *vEulerRD){ return bno055_convert_double_euler_r_deg(v_euler_r_d); } 
		STD_RET ConvertDoubleEulerHRad(double *vEulerHD){ return bno055_convert_double_euler_h_rad(v_euler_h_d); } 
		STD_RET ConvertDoubleEulerPRad(double *vEulerPD){ return bno055_convert_double_euler_p_rad(v_euler_p_d); } 
		STD_RET ConvertDoubleEulerRRad(double *vEulerRD){ return bno055_convert_double_euler_r_rad(v_euler_r_d); } 
		STD_RET ConvertDoubleEulerHprDeg(struct bno055EulerDoubleT *eulerHpr){ return bno055_convert_double_euler_hpr_deg(euler_hpr); } 
		STD_RET ConvertDoubleEulerHprRad(struct bno055EulerDoubleT *eulerHpr){ return bno055_convert_double_euler_hpr_rad(euler_hpr); } 
		STD_RET ConvertDoubleLinearAccelXMsq(double *vLinearAccelXD){ return bno055_convert_double_linear_accel_x_msq(v_linear_accel_x_d); } 
		STD_RET ConvertDoubleLinearAccelYMsq(double *vLinearAccelYD){ return bno055_convert_double_linear_accel_y_msq(v_linear_accel_y_d); } 
		STD_RET ConvertDoubleLinearAccelZMsq(double *vLinearAccelZD){ return bno055_convert_double_linear_accel_z_msq(v_linear_accel_z_d); } 
		STD_RET ConvertDoubleLinearAccelXyzMsq(struct bno055LinearAccelDoubleT *linearAccelXyz){ return bno055_convert_double_linear_accel_xyz_msq(linear_accel_xyz); } 
		STD_RET ConvertGravityDoubleXMsq(double *vGravityXD){ return bno055_convert_gravity_double_x_msq(v_gravity_x_d); } 
		STD_RET ConvertGravityDoubleYMsq(double *vGravityYD){ return bno055_convert_gravity_double_y_msq(v_gravity_y_d); } 
		STD_RET ConvertGravityDoubleZMsq(double *vGravityZD){ return bno055_convert_gravity_double_z_msq(v_gravity_z_d); } 
		STD_RET ConvertDoubleGravityXyzMsq(struct bno055GravityDoubleT *gravityXyz){ return bno055_convert_double_gravity_xyz_msq(gravity_xyz); } 
		STD_RET ConvertDoubleTempFahrenheit(double *vTempD){ return bno055_convert_double_temp_fahrenheit(v_temp_d); } 
		STD_RET ConvertDoubleTempCelsius(double *vTempD){ return bno055_convert_double_temp_celsius(v_temp_d); } 
		STD_RET GetMagCalibStat(u8 *vMagCalibU8){ return bno055_get_mag_calib_stat(v_mag_calib_u_8); } 
		STD_RET GetAccelCalibStat(u8 *vAccelCalibU8){ return bno055_get_accel_calib_stat(v_accel_calib_u_8); } 
		STD_RET GetGyroCalibStat(u8 *vGyroCalibU8){ return bno055_get_gyro_calib_stat(v_gyro_calib_u_8); } 
		STD_RET GetSysCalibStat(u8 *vSysCalibU8){ return bno055_get_sys_calib_stat(v_sys_calib_u_8); } 
		STD_RET GetSelftestAccel(u8 *vSelftestAccelU8){ return bno055_get_selftest_accel(v_selftest_accel_u_8); } 
		STD_RET GetSelftestMag(u8 *vSelftestMagU8){ return bno055_get_selftest_mag(v_selftest_mag_u_8); } 
		STD_RET GetSelftestGyro(u8 *vSelftestGyroU8){ return bno055_get_selftest_gyro(v_selftest_gyro_u_8); } 
		STD_RET GetSelftestMcu(u8 *vSelftestMcuU8){ return bno055_get_selftest_mcu(v_selftest_mcu_u_8); } 
		STD_RET GetIntrStatGyroAnyMotion(u8 *vGyroAnyMotionU8){ return bno055_get_intr_stat_gyro_any_motion(v_gyro_any_motion_u_8); } 
		STD_RET GetIntrStatGyroHighrate(u8 *vGyroHighrateU8){ return bno055_get_intr_stat_gyro_highrate(v_gyro_highrate_u_8); } 
		STD_RET GetIntrStatAccelHighG(u8 *vAccelHighGU8){ return bno055_get_intr_stat_accel_high_g(v_accel_high_g_u_8); } 
		STD_RET GetIntrStatAccelAnyMotion(u8 *vAccelAnyMotionU8){ return bno055_get_intr_stat_accel_any_motion(v_accel_any_motion_u_8); } 
		STD_RET GetIntrStatAccelNoMotion(u8 *vAccelNoMotionU8){ return bno055_get_intr_stat_accel_no_motion(v_accel_no_motion_u_8); } 
		STD_RET GetStatMainClk(u8 *vStatMainClkU8){ return bno055_get_stat_main_clk(v_stat_main_clk_u_8); } 
		STD_RET GetSysStatCode(u8 *vSysStatU8){ return bno055_get_sys_stat_code(v_sys_stat_u_8); } 
		STD_RET GetSysErrorCode(u8 *vSysErrorU8){ return bno055_get_sys_error_code(v_sys_error_u_8); } 
		STD_RET GetAccelUnit(u8 *vAccelUnitU8){ return bno055_get_accel_unit(v_accel_unit_u_8); } 
		STD_RET SetAccelUnit(u8 vAccelUnitU8){ return bno055_set_accel_unit(v_accel_unit_u_8); } 
		STD_RET GetGyroUnit(u8 *vGyroUnitU8){ return bno055_get_gyro_unit(v_gyro_unit_u_8); } 
		STD_RET SetGyroUnit(u8 vGyroUnitU8){ return bno055_set_gyro_unit(v_gyro_unit_u_8); } 

		STD_RET GetEulerUnit(u8 *vEulerUnitU8){ return bno055_get_euler_unit(v_euler_unit_u_8); } 
		STD_RET SetEulerUnit(u8 vEulerUnitU8){ return bno055_set_euler_unit(v_euler_unit_u_8); } 

		STD_RET GetTiltUnit(u8 *vTiltUnitU8){ return bno055_get_tilt_unit(v_tilt_unit_u_8); } 
		STD_RET SetTiltUnit(u8 vTiltUnitU8){ return bno055_set_tilt_unit(v_tilt_unit_u_8); } 
		STD_RET GetTempUnit(u8 *vTempUnitU8){ return bno055_get_temp_unit(v_temp_unit_u_8); } 
		STD_RET SetTempUnit(u8 vTempUnitU8){ return bno055_set_temp_unit(v_temp_unit_u_8); } 
		STD_RET GetDataOutputFormat(u8 *vDataOutputFormatU8){ return bno055_get_data_output_format(v_data_output_format_u_8); } 
		STD_RET SetDataOutputFormat(u8 vDataOutputFormatU8){ return bno055_set_data_output_format(v_data_output_format_u_8); } 
		STD_RET GetOperationMode(u8 *vOperationModeU8){ return bno055_get_operation_mode(v_operation_mode_u_8); } 
		STD_RET SetOperationMode(u8 vOperationModeU8){ return bno055_set_operation_mode(v_operation_mode_u_8); } 

		STD_RET GetPowerMode(u8 *vPowerModeU8){ return bno055_get_power_mode(v_power_mode_u_8); } 
		STD_RET SetPowerMode(u8 vPowerModeU8){ return bno055_set_power_mode(v_power_mode_u_8); } 

		STD_RET GetIntrRst(u8 *vIntrRstU8){ return bno055_get_intr_rst(v_intr_rst_u_8); } 
		STD_RET SetIntrRst(u8 vIntrRstU8){ return bno055_set_intr_rst(v_intr_rst_u_8); } 

		STD_RET GetClkSrc(u8 *vClkSrcU8){ return bno055_get_clk_src(v_clk_src_u_8); } 
		STD_RET SetClkSrc(u8 vClkSrcU8){ return bno055_set_clk_src(v_clk_src_u_8); } 

		STD_RET GetSysRst(u8 *vSysRstU8){ return bno055_get_sys_rst(v_sys_rst_u_8); } 
		STD_RET SetSysRst(u8 vSysRstU8){ return bno055_set_sys_rst(v_sys_rst_u_8); } 

		STD_RET GetSelftest(u8 *vSelftestU8){ return bno055_get_selftest(v_selftest_u_8); } 
		STD_RET SetSelftest(u8 vSelftestU8){ return bno055_set_selftest(v_selftest_u_8); } 

		STD_RET GetTempSource(u8 *vTempSourceU8){ return bno055_get_temp_source(v_temp_source_u_8); } 
		STD_RET SetTempSource(u8 vTempSourceU8){ return bno055_set_temp_source(v_temp_source_u_8); } 

		STD_RET GetAxisRemapValue(u8 *vRemapAxisU8){ return bno055_get_axis_remap_value(v_remap_axis_u_8); } 
		STD_RET SetAxisRemapValue(u8 vRemapAxisU8){ return bno055_set_axis_remap_value(v_remap_axis_u_8); } 
		STD_RET GetRemapXSign(u8 *vRemapXSignU8){ return bno055_get_remap_x_sign(v_remap_x_sign_u_8); } 
		STD_RET SetRemapXSign(u8 vRemapXSignU8){ return bno055_set_remap_x_sign(v_remap_x_sign_u_8); } 
		STD_RET GetRemapYSign(u8 *vRemapYSignU8){ return bno055_get_remap_y_sign(v_remap_y_sign_u_8); } 
		STD_RET SetRemapYSign(u8 vRemapYSignU8){ return bno055_set_remap_y_sign(v_remap_y_sign_u_8); } 
		STD_RET GetRemapZSign(u8 *vRemapZSignU8){ return bno055_get_remap_z_sign(v_remap_z_sign_u_8); } 
		STD_RET SetRemapZSign(u8 vRemapZSignU8){ return bno055_set_remap_z_sign(v_remap_z_sign_u_8); } 
		STD_RET ReadSicMatrix(struct bno055SicMatrixT  *sicMatrix){ return bno055_read_sic_matrix(sic_matrix); } 
		STD_RET WriteSicMatrix(struct bno055SicMatrixT  *sicMatrix){ return bno055_write_sic_matrix(sic_matrix); } 
		STD_RET ReadAccelOffset(struct bno055AccelOffsetT  *accelOffset){ return bno055_read_accel_offset(accel_offset); } 
		STD_RET WriteAccelOffset(struct bno055AccelOffsetT  *accelOffset){ return bno055_write_accel_offset(accel_offset); } 
		STD_RET ReadMagOffset(struct bno055MagOffsetT  *magOffset){ return bno055_read_mag_offset(mag_offset); } 
		STD_RET WriteMagOffset(struct bno055MagOffsetT  *magOffset){ return bno055_write_mag_offset(mag_offset); } 
		STD_RET ReadGyroOffset(struct bno055GyroOffsetT  *gyroOffset){ return bno055_read_gyro_offset(gyro_offset); } 
		STD_RET WriteGyroOffset(struct bno055GyroOffsetT *gyroOffset){ return bno055_write_gyro_offset(gyro_offset); } 
		STD_RET GetAccelRange(u8 *vAccelRangeU8){ return bno055_get_accel_range(v_accel_range_u_8); } 
		STD_RET SetAccelRange(u8 vAccelRangeU8){ return bno055_set_accel_range(v_accel_range_u_8); } 
		STD_RET GetAccelBw(u8 *vAccelBwU8){ return bno055_get_accel_bw(v_accel_bw_u_8); } 
		STD_RET SetAccelBw(u8 vAccelBwU8){ return bno055_set_accel_bw(v_accel_bw_u_8); } 
		STD_RET GetAccelPowerMode(u8 *vAccelPowerModeU8){ return bno055_get_accel_power_mode(v_accel_power_mode_u_8); } 
		STD_RET SetAccelPowerMode(u8 vAccelPowerModeU8){ return bno055_set_accel_power_mode(v_accel_power_mode_u_8); } 
		STD_RET GetMagDataOutputRate(u8 *vMagDataOutputRateU8){ return bno055_get_mag_data_output_rate(v_mag_data_output_rate_u_8); } 
		STD_RET SetMagDataOutputRate(u8 vMagDataOutputRateU8){ return bno055_set_mag_data_output_rate(v_mag_data_output_rate_u_8); } 
		STD_RET GetMagOperationMode(u8 *vMagOperationModeU8){ return bno055_get_mag_operation_mode(v_mag_operation_mode_u_8); } 
		STD_RET SetMagOperationMode(u8 vMagOperationModeU8){ return bno055_set_mag_operation_mode(v_mag_operation_mode_u_8); } 
		STD_RET GetMagPowerMode(u8 *vMagPowerModeU8){ return bno055_get_mag_power_mode(v_mag_power_mode_u_8); } 
		STD_RET SetMagPowerMode(u8 vMagPowerModeU8){ return bno055_set_mag_power_mode(v_mag_power_mode_u_8); } 
		STD_RET GetGyroRange(u8 *vGyroRangeU8){ return bno055_get_gyro_range(v_gyro_range_u_8); } 
		STD_RET SetGyroRange(u8 vGyroRangeU8){ return bno055_set_gyro_range(v_gyro_range_u_8); } 
		STD_RET GetGyroBw(u8 *vGyroBwU8){ return bno055_get_gyro_bw(v_gyro_bw_u_8); } 
		STD_RET SetGyroBw(u8 vGyroBwU8){ return bno055_set_gyro_bw(v_gyro_bw_u_8); } 
		STD_RET GetGyroPowerMode(u8 *vGyroPowerModeU8){ return bno055_get_gyro_power_mode(v_gyro_power_mode_u_8); } 
		STD_RET SetGyroPowerMode(u8 vGyroPowerModeU8){ return bno055_set_gyro_power_mode(v_gyro_power_mode_u_8); } 
		STD_RET GetAccelSleepTmrMode(u8 *vSleepTmrU8){ return bno055_get_accel_sleep_tmr_mode(v_sleep_tmr_u_8); } 
		STD_RET SetAccelSleepTmrMode(u8 vSleepTmrU8){ return bno055_set_accel_sleep_tmr_mode(v_sleep_tmr_u_8); } 
		STD_RET GetAccelSleepDurn(u8 *vSleepDurnU8){ return bno055_get_accel_sleep_durn(v_sleep_durn_u_8); } 
		STD_RET SetAccelSleepDurn(u8 vSleepDurnU8){ return bno055_set_accel_sleep_durn(v_sleep_durn_u_8); } 
		STD_RET GetGyroSleepDurn(u8 *vSleepDurnU8){ return bno055_get_gyro_sleep_durn(v_sleep_durn_u_8); } 
		STD_RET SetGyroSleepDurn(u8 sleepDurn){ return bno055_set_gyro_sleep_durn(sleep_durn); } 
		STD_RET GetGyroAutoSleepDurn(u8 *vAutoSleepDurnU8){ return bno055_get_gyro_auto_sleep_durn(v_auto_sleep_durn_u_8); } 
		STD_RET GyroSetAutoSleepDurn(u8 vAutoSleepDurnU8, u8 bw){ return bno055_gyro_set_auto_sleep_durn(v_auto_sleep_durn_u_8, bw); } 
		STD_RET GetMagSleepMode(u8 *vSleepModeU8){ return bno055_get_mag_sleep_mode(v_sleep_mode_u_8); } 
		STD_RET SetMagSleepMode(u8 vSleepModeU8){ return bno055_set_mag_sleep_mode(v_sleep_mode_u_8); } 
		STD_RET GetMagSleepDurn(u8 *vSleepDurnU8){ return bno055_get_mag_sleep_durn(v_sleep_durn_u_8); } 
		STD_RET SetMagSleepDurn(u8 vSleepDurnU8){ return bno055_set_mag_sleep_durn(v_sleep_durn_u_8); } 
		STD_RET GetIntrMaskGyroAnyMotion(u8 *vGyroAnyMotionU8){ return bno055_get_intr_mask_gyro_any_motion(v_gyro_any_motion_u_8); } 
		STD_RET SetIntrMaskGyroAnyMotion(u8 vGyroAnyMotionU8){ return bno055_set_intr_mask_gyro_any_motion(v_gyro_any_motion_u_8); } 
		STD_RET GetIntrMaskGyroHighrate(u8 *vGyroHighrateU8){ return bno055_get_intr_mask_gyro_highrate(v_gyro_highrate_u_8); } 
		STD_RET SetIntrMaskGyroHighrate(u8 vGyroHighrateU8){ return bno055_set_intr_mask_gyro_highrate(v_gyro_highrate_u_8); } 
		STD_RET GetIntrMaskAccelHighG(u8 *vAccelHighGU8){ return bno055_get_intr_mask_accel_high_g(v_accel_high_g_u_8); } 
		STD_RET SetIntrMaskAccelHighG(u8 vAccelHighGU8){ return bno055_set_intr_mask_accel_high_g(v_accel_high_g_u_8); } 
		STD_RET GetIntrMaskAccelAnyMotion(u8 *vAccelAnyMotionU8){ return bno055_get_intr_mask_accel_any_motion(v_accel_any_motion_u_8); } 
		STD_RET SetIntrMaskAccelAnyMotion(u8 vAccelAnyMotionU8){ return bno055_set_intr_mask_accel_any_motion(v_accel_any_motion_u_8); } 
		STD_RET GetIntrMaskAccelNoMotion(u8 *vAccelNomotionU8){ return bno055_get_intr_mask_accel_no_motion(v_accel_nomotion_u_8); } 
		STD_RET SetIntrMaskAccelNoMotion(u8 vAccelNomotionU8){ return bno055_set_intr_mask_accel_no_motion(v_accel_nomotion_u_8); } 
		STD_RET GetIntrGyroAnyMotion(u8 *vGyroAnyMotionU8){ return bno055_get_intr_gyro_any_motion(v_gyro_any_motion_u_8); } 
		STD_RET SetIntrGyroAnyMotion(u8 vGyroAnyMotionU8){ return bno055_set_intr_gyro_any_motion(v_gyro_any_motion_u_8); } 
		STD_RET GetIntrGyroHighrate(u8 *vGyroHighrateU8){ return bno055_get_intr_gyro_highrate(v_gyro_highrate_u_8); } 
		STD_RET SetIntrGyroHighrate(u8 vGyroHighrateU8){ return bno055_set_intr_gyro_highrate(v_gyro_highrate_u_8); } 
		STD_RET GetIntrAccelHighG(u8 *vAccelHighGU8){ return bno055_get_intr_accel_high_g(v_accel_high_g_u_8); } 
		STD_RET SetIntrAccelHighG(u8 vAccelHighGU8){ return bno055_set_intr_accel_high_g(v_accel_high_g_u_8); } 
		STD_RET GetIntrAccelAnyMotion(u8 *vAccelAnyMotionU8){ return bno055_get_intr_accel_any_motion(v_accel_any_motion_u_8); } 
		STD_RET SetIntrAccelAnyMotion(u8 vAccelAnyMotionU8){ return bno055_set_intr_accel_any_motion(v_accel_any_motion_u_8); } 
		STD_RET GetIntrAccelNoMotion(u8 *vAccelNomotionU8){ return bno055_get_intr_accel_no_motion(v_accel_nomotion_u_8); } 
		STD_RET SetIntrAccelNoMotion(u8 vAccelNomotionU8){ return bno055_set_intr_accel_no_motion(v_accel_nomotion_u_8); } 
		STD_RET GetAccelAnyMotionThres(u8 *vAccelAnyMotionThresU8){ return bno055_get_accel_any_motion_thres(v_accel_any_motion_thres_u_8); } 
		STD_RET SetAccelAnyMotionThres(u8 vAccelAnyMotionThresU8){ return bno055_set_accel_any_motion_thres(v_accel_any_motion_thres_u_8); } 
		STD_RET GetAccelAnyMotionDurn(u8 *vAccelAnyMotionDurnU8){ return bno055_get_accel_any_motion_durn(v_accel_any_motion_durn_u_8); } 
		STD_RET SetAccelAnyMotionDurn(u8 vAccelAnyMotionDurnU8){ return bno055_set_accel_any_motion_durn(v_accel_any_motion_durn_u_8); } 
		STD_RET GetAccelAnyMotionNoMotionAxisEnable(u8 vChannelU8, u8 *vDataU8){ return bno055_get_accel_any_motion_no_motion_axis_enable(v_channel_u_8, v_data_u_8); } 
		STD_RET SetAccelAnyMotionNoMotionAxisEnable(u8 vChannelU8, u8 vDataU8){ return bno055_set_accel_any_motion_no_motion_axis_enable(v_channel_u_8, v_data_u_8); } 
		STD_RET GetAccelHighGAxisEnable(u8 vChannelU8, u8 *vDataU8){ return bno055_get_accel_high_g_axis_enable(v_channel_u_8, v_data_u_8); } 
		STD_RET SetAccelHighGAxisEnable(u8 vChannelU8, u8 vDataU8){ return bno055_set_accel_high_g_axis_enable(v_channel_u_8, v_data_u_8); } 
		STD_RET GetAccelHighGDurn(u8 *vAccelHighGDurnU8){ return bno055_get_accel_high_g_durn(*v_accel_high_g_durn_u_8); } 
		STD_RET SetAccelHighGDurn(u8 vAccelHighGDurnU8){ return bno055_set_accel_high_g_durn(v_accel_high_g_durn_u_8); } 
		STD_RET GetAccelHighGThres(u8 *vAccelHighGThresU8){ return bno055_get_accel_high_g_thres(v_accel_high_g_thres_u_8); } 
		STD_RET SetAccelHighGThres(u8 vAccelHighGThresU8){ return bno055_set_accel_high_g_thres(v_accel_high_g_thres_u_8); } 
		STD_RET GetAccelSlowNoMotionThres(u8 *vAccelSlowNoMotionThresU8){ return bno055_get_accel_slow_no_motion_thres(v_accel_slow_no_motion_thres_u_8); } 
		STD_RET SetAccelSlowNoMotionThres(u8 vAccelSlowNoMotionThresU8){ return bno055_set_accel_slow_no_motion_thres(v_accel_slow_no_motion_thres_u_8); } 
		STD_RET GetAccelSlowNoMotionEnable(u8 *vAccelSlowNoMotionEnU8){ return bno055_get_accel_slow_no_motion_enable(v_accel_slow_no_motion_en_u_8); } 
		STD_RET SetAccelSlowNoMotionEnable(u8 vAccelSlowNoMotionEnU8){ return bno055_set_accel_slow_no_motion_enable(v_accel_slow_no_motion_en_u_8); } 
		STD_RET GetAccelSlowNoMotionDurn(u8 *vAccelSlowNoMotionDurnU8){ return bno055_get_accel_slow_no_motion_durn(v_accel_slow_no_motion_durn_u_8); } 
		STD_RET SetAccelSlowNoMotionDurn(u8 vAccelSlowNoMotionDurnU8){ return bno055_set_accel_slow_no_motion_durn(v_accel_slow_no_motion_durn_u_8); } 
		STD_RET GetGyroAnyMotionAxisEnable(u8 vChannelU8, u8 *vDataU8){ return bno055_get_gyro_any_motion_axis_enable(v_channel_u_8, v_data_u_8); } 
		STD_RET SetGyroAnyMotionAxisEnable(u8 vChannelU8, u8  vDataU8){ return bno055_set_gyro_any_motion_axis_enable(v_channel_u_8, v_data_u_8); } 
		STD_RET GetGyroHighrateAxisEnable(u8 vChannelU8, u8 *vDataU8){ return bno055_get_gyro_highrate_axis_enable(v_channel_u_8, v_data_u_8); } 
		STD_RET SetGyroHighrateAxisEnable(u8 vChannelU8, u8 vDataU8){ return bno055_set_gyro_highrate_axis_enable(v_channel_u_8, v_data_u_8); } 
		STD_RET GetGyroAnyMotionFilter(u8 *vGyroAnyMotionFilterU8){ return bno055_get_gyro_any_motion_filter(v_gyro_any_motion_filter_u_8); } 
		STD_RET SetGyroAnyMotionFilter(u8 vGyroAnyMotionFilterU8){ return bno055_set_gyro_any_motion_filter(v_gyro_any_motion_filter_u_8); } 
		STD_RET GetGyroHighrateFilter(u8 *vGyroHighrateFilterU8){ return bno055_get_gyro_highrate_filter(v_gyro_highrate_filter_u_8); } 
		STD_RET SetGyroHighrateFilter(u8 vGyroHighrateFilterU8){ return bno055_set_gyro_highrate_filter(v_gyro_highrate_filter_u_8); } 
		STD_RET GetGyroHighrateXThres(u8 *vGyroHighrateXThresU8){ return bno055_get_gyro_highrate_x_thres(v_gyro_highrate_x_thres_u_8); } 
		STD_RET SetGyroHighrateXThres(u8 vGyroHighrateXThresU8){ return bno055_set_gyro_highrate_x_thres(v_gyro_highrate_x_thres_u_8); } 
		STD_RET GetGyroHighrateXHyst(u8 *vGyroHighrateXHystU8){ return bno055_get_gyro_highrate_x_hyst(v_gyro_highrate_x_hyst_u_8); } 
		STD_RET SetGyroHighrateXHyst(u8 vGyroHighrateXHystU8){ return bno055_set_gyro_highrate_x_hyst(v_gyro_highrate_x_hyst_u_8); } 
		STD_RET GetGyroHighrateXDurn(u8 *vGyroHighrateXDurnU8){ return bno055_get_gyro_highrate_x_durn(v_gyro_highrate_x_durn_u_8); } 
		STD_RET SetGyroHighrateXDurn(u8 vGyroHighrateXDurnU8){ return bno055_set_gyro_highrate_x_durn(v_gyro_highrate_x_durn_u_8); } 
		STD_RET GetGyroHighrateYThres(u8 *vGyroHighrateYThresU8){ return bno055_get_gyro_highrate_y_thres(v_gyro_highrate_y_thres_u_8); } 
		STD_RET SetGyroHighrateYThres(u8 vGyroHighrateYThresU8){ return bno055_set_gyro_highrate_y_thres(v_gyro_highrate_y_thres_u_8); } 
		STD_RET GetGyroHighrateYHyst(u8 *vGyroHighrateYHystU8){ return bno055_get_gyro_highrate_y_hyst(v_gyro_highrate_y_hyst_u_8); } 
		STD_RET SetGyroHighrateYHyst(u8 vGyroHighrateYHystU8){ return bno055_set_gyro_highrate_y_hyst(v_gyro_highrate_y_hyst_u_8); } 
		STD_RET GetGyroHighrateYDurn(u8 *vGyroHighrateYDurnU8){ return bno055_get_gyro_highrate_y_durn(v_gyro_highrate_y_durn_u_8); } 
		STD_RET SetGyroHighrateYDurn(u8 vGyroHighrateYDurnU8){ return bno055_set_gyro_highrate_y_durn(v_gyro_highrate_y_durn_u_8); } 
		STD_RET GetGyroHighrateZThres(u8 *vGyroHighrateZThresU8){ return bno055_get_gyro_highrate_z_thres(v_gyro_highrate_z_thres_u_8); } 
		STD_RET SetGyroHighrateZThres(u8 vGyroHighrateZThresU8){ return bno055_set_gyro_highrate_z_thres(v_gyro_highrate_z_thres_u_8); } 
		STD_RET GetGyroHighrateZHyst(u8 *vGyroHighrateZHystU8){ return bno055_get_gyro_highrate_z_hyst(v_gyro_highrate_z_hyst_u_8); } 
		STD_RET SetGyroHighrateZHyst(u8 vGyroHighrateZHystU8){ return bno055_set_gyro_highrate_z_hyst(v_gyro_highrate_z_hyst_u_8); } 
		STD_RET GetGyroHighrateZDurn(u8 *vGyroHighrateZDurnU8){ return bno055_get_gyro_highrate_z_durn(v_gyro_highrate_z_durn_u_8); } 
		STD_RET SetGyroHighrateZDurn(u8 vGyroHighrateZDurnU8){ return bno055_set_gyro_highrate_z_durn(v_gyro_highrate_z_durn_u_8); } 
		STD_RET GetGyroAnyMotionThres(u8 *vGyroAnyMotionThresU8){ return bno055_get_gyro_any_motion_thres(v_gyro_any_motion_thres_u_8); } 
		STD_RET SetGyroAnyMotionThres(u8 vGyroAnyMotionThresU8){ return bno055_set_gyro_any_motion_thres(v_gyro_any_motion_thres_u_8); } 
		STD_RET GetGyroAnyMotionSlopeSamples(u8 *vGyroAnyMotionSlopeSamplesU8){ return bno055_get_gyro_any_motion_slope_samples(v_gyro_any_motion_slope_samples_u_8); } 
		STD_RET SetGyroAnyMotionSlopeSamples(u8 vGyroAnyMotionSlopeSamplesU8){ return bno055_set_gyro_any_motion_slope_samples(v_gyro_any_motion_slope_samples_u_8); } 
		STD_RET GetGyroAnyMotionAwakeDurn(u8 *vGyroAwakeDurnU8){ return bno055_get_gyro_any_motion_awake_durn(v_gyro_awake_durn_u_8); } 
		STD_RET SetGyroAnyMotionAwakeDurn(u8 vGyroAwakeDurnU8){ return bno055_set_gyro_any_motion_awake_durn(v_gyro_awake_durn_u_8); } 


};

#endif