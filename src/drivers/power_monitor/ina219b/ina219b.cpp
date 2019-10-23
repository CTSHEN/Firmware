/****************************************************************************
 *
 *   Copyright (c) 2019 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ina219b.cpp
 * @author David Sidrane <david_s5@usa.net>
 * @ modified by C.T. Shen from ina226.cpp
 *
 * Driver for the I2C attached INA219b (another one)
 */
#define INA219b_RAW // remove this

#include <string.h>

#include <px4_config.h>
#include <px4_getopt.h>
#include <drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>
#include <uORB/uORB.h>
#include <uORB/topics/power_monitor.h>
#include <px4_work_queue/ScheduledWorkItem.hpp>

/* Configuration Constants */
//#define INA219b_BUS_DEFAULT		                PX4_I2C_BUS_EXPANSION
#define INA219b_BASEADDR 	                    0x40 /* 7-bit address. 1000000 ==> 0x40 (solar_in) */
							 /*address pins A0 and A1 are both 0, CTSHEN */

/* INA219b Registers addresses */
#define INA219b_REG_CONFIGURATION             (0x00)
#define INA219b_REG_SHUNTVOLTAGE              (0x01)
#define INA219b_REG_BUSVOLTAGE                (0x02)
#define INA219b_REG_POWER                     (0x03)
#define INA219b_REG_CURRENT                   (0x04)
#define INA219b_REG_CALIBRATION               (0x05)








/* INA219b Configuration Register */
#define INA219b_MODE_SHIFTS                   (0)
#define INA219b_MODE_MASK                     (7 << INA219b_MODE_SHIFTS)
#define INA219b_MODE_SHUTDOWN                 (0 << INA219b_MODE_SHIFTS)
#define INA219b_MODE_SHUNT_TRIG               (1 << INA219b_MODE_SHIFTS)
#define INA219b_MODE_BUS_TRIG                 (2 << INA219b_MODE_SHIFTS)
#define INA219b_MODE_SHUNT_BUS_TRIG           (3 << INA219b_MODE_SHIFTS)
#define INA219b_MODE_ADC_OFF                  (4 << INA219b_MODE_SHIFTS)
#define INA219b_MODE_SHUNT_CONT               (5 << INA219b_MODE_SHIFTS)
#define INA219b_MODE_BUS_CONT                 (6 << INA219b_MODE_SHIFTS)
#define INA219b_MODE_SHUNT_BUS_CONT           (7 << INA219b_MODE_SHIFTS)

/*#define INA219b_VSHCT_SHIFTS                  (3)
#define INA219b_VSHCT_MASK                    (7 << INA219b_VSHCT_SHIFTS)
#define INA219b_VSHCT_140US                   (0 << INA219b_VSHCT_SHIFTS)
#define INA219b_VSHCT_204US                   (1 << INA219b_VSHCT_SHIFTS)
#define INA219b_VSHCT_332US                   (2 << INA219b_VSHCT_SHIFTS)
#define INA219b_VSHCT_588US                   (3 << INA219b_VSHCT_SHIFTS)
#define INA219b_VSHCT_1100US                  (4 << INA219b_VSHCT_SHIFTS)
#define INA219b_VSHCT_2116US                  (5 << INA219b_VSHCT_SHIFTS)
#define INA219b_VSHCT_4156US                  (6 << INA219b_VSHCT_SHIFTS)
#define INA219b_VSHCT_8244US                  (7 << INA219b_VSHCT_SHIFTS)

#define INA219b_VBUSCT_SHIFTS                 (6)
#define INA219b_VBUSCT_MASK                   (7 << INA219b_VBUSCT_SHIFTS)
#define INA219b_VBUSCT_140US                  (0 << INA219b_VBUSCT_SHIFTS)
#define INA219b_VBUSCT_204US                  (1 << INA219b_VBUSCT_SHIFTS)
#define INA219b_VBUSCT_332US                  (2 << INA219b_VBUSCT_SHIFTS)
#define INA219b_VBUSCT_588US                  (3 << INA219b_VBUSCT_SHIFTS)
#define INA219b_VBUSCT_1100US                 (4 << INA219b_VBUSCT_SHIFTS)
#define INA219b_VBUSCT_2116US                 (5 << INA219b_VBUSCT_SHIFTS)
#define INA219b_VBUSCT_4156US                 (6 << INA219b_VBUSCT_SHIFTS)
#define INA219b_VBUSCT_8244US                 (7 << INA219b_VBUSCT_SHIFTS)

#define INA219b_AVERAGES_SHIFTS                (9)
#define INA219b_AVERAGES_MASK                  (7 << INA219b_AVERAGES_SHIFTS)
#define INA219b_AVERAGES_1                     (0 << INA219b_AVERAGES_SHIFTS)
#define INA219b_AVERAGES_4                     (1 << INA219b_AVERAGES_SHIFTS)
#define INA219b_AVERAGES_16                    (2 << INA219b_AVERAGES_SHIFTS)
#define INA219b_AVERAGES_64                    (3 << INA219b_AVERAGES_SHIFTS)
#define INA219b_AVERAGES_128                   (4 << INA219b_AVERAGES_SHIFTS)
#define INA219b_AVERAGES_256                   (5 << INA219b_AVERAGES_SHIFTS)
#define INA219b_AVERAGES_512                   (6 << INA219b_AVERAGES_SHIFTS)
#define INA219b_AVERAGES_1024                  (7 << INA219b_AVERAGES_SHIFTS)
*/

#define INA219b_SADC_SHIFTS			(3)
#define INA219b_SADC_9bit			(0 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_10bit			(1 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_11bit			(2 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_12bit			(3 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_12bit2			(8 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_2				(9 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_4				(10 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_8				(11 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_16				(12 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_32				(13 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_64				(14 << INA219b_SADC_SHIFTS)
#define INA219b_SADC_128				(15 << INA219b_SADC_SHIFTS)

#define INA219b_BADC_SHIFTS			(7)
#define INA219b_BADC_9bit			(0 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_10bit			(1 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_11bit			(2 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_12bit			(3 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_12bit2			(8 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_2				(9 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_4				(10 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_8				(11 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_16				(12 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_32				(13 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_64				(14 << INA219b_BADC_SHIFTS)
#define INA219b_BADC_128				(15 << INA219b_BADC_SHIFTS)

#define INA219b_PG_SHIFTS			(11)
#define INA219b_PG_1				(0 << INA219b_PG_SHIFTS)
#define INA219b_PG_2				(1 << INA219b_PG_SHIFTS)
#define INA219b_PG_4				(2 << INA219b_PG_SHIFTS)
#define INA219b_PG_8				(3 << INA219b_PG_SHIFTS)

#define INA219b_BRNG_SHIFTS			(13)
#define INA219b_BRNG_16				(0 << INA219b_BRNG_SHIFTS)
#define INA219b_BRNG_32				(1 << INA219b_BRNG_SHIFTS)

#define INA219b_CONFIG (INA219b_MODE_SHUNT_BUS_CONT | INA219b_SADC_12bit | INA219b_BADC_12bit | INA219b_PG_8 | INA219b_BRNG_32)

#define INA219b_RST                            (1 << 15)

/* INA219b Enable / Mask Register */
/*
#define INA219b_LEN                           (1 << 0)
#define INA219b_APOL                          (1 << 1)
#define INA219b_OVF                           (1 << 2)
#define INA219b_CVRF                          (1 << 3)
#define INA219b_AFF                           (1 << 4)

#define INA219b_CNVR                          (1 << 10)
#define INA219b_POL                           (1 << 11)
#define INA219b_BUL                           (1 << 12)
#define INA219b_BOL                           (1 << 13)
#define INA219b_SUL                           (1 << 14)
#define INA219b_SOL                           (1 << 15)
*/
#define INA219b_CONVERSION_INTERVAL 	          (100000-7) /* 100 ms / 10 Hz */
#define MAX_CURRENT                           8.0f    /* +-8 Amps CTSHEN*/
#define DN_MAX                                32768.0f  /* 2^15 */
#define INA219b_CONST                          0.04096f  /* is an internal fixed value used to ensure scaling is maintained properly  */
#define INA219b_SHUNT                          0.01f   /* Shunt is 0.01 Ohm CTHSHEN */
#define INA219b_VSCALE                         0.004f  /* LSB of voltage is 4 mV  CTSHEN */

#define swap16(w)                       __builtin_bswap16((w))

static constexpr uint8_t INA219b_BUS_DEFAULT = PX4_I2C_BUS_EXPANSION;

class INA219b : public device::I2C, px4::ScheduledWorkItem
{
public:
	INA219b(int bus , int address = INA219b_BASEADDR);
	virtual ~INA219b();

	virtual int 		  init();

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void				      print_info();

protected:
	virtual int	  		probe();

private:
	bool			        _sensor_ok{false};
	int				        _measure_interval{0};
	bool			        _collect_phase{false};

	orb_advert_t		  _power_monitor_topic{nullptr};

	perf_counter_t		_sample_perf;
	perf_counter_t		_comms_errors;

	int16_t           _bus_volatage{0};
	int16_t           _power{-1};
	int16_t           _current{-1};
	int16_t           _shunt{0};
	int16_t           _cal{0};
	bool              _mode_trigged{false};

	float             _max_current{MAX_CURRENT};
	float             _rshunt{INA219b_SHUNT};
	uint16_t          _config{INA219b_CONFIG};
	float             _current_lsb{_max_current / DN_MAX};
	float             _power_lsb{20.0f * _current_lsb};

	/**
	* Test whetpower_monitorhe device supported by the driver is present at a
	* specific address.
	*
	* @param address	The I2C bus address to read or write.
	* @return			.
	*/
	int               read(uint8_t address);
	int               write(uint8_t address, uint16_t data);

	/**
	* Initialise the automatic measurement state machine and start it.
	*
	* @note This function is called at open and error time.  It might make sense
	*       to make it more aggressive about resetting the bus in case of errors.
	*/
	void				      start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void				      stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void				     Run() override;

	int					     measure();
	int					     collect();

};

/*
 * Driver 'main' command.
 */
extern "C" __EXPORT int ina219b_main(int argc, char *argv[]);

INA219b::INA219b(int bus, int address) :
	I2C("INA219b", nullptr, bus, address, 100000),
	ScheduledWorkItem(px4::device_bus_to_wq(get_device_id())),
	_sample_perf(perf_alloc(PC_ELAPSED, "ina219b_read")),
	_comms_errors(perf_alloc(PC_COUNT, "ina219b_com_err"))
{
	float fvalue = MAX_CURRENT;
	_max_current = fvalue;
	param_t ph = param_find("INA219b_CURRENT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_max_current = fvalue;
	}

	fvalue = INA219b_SHUNT;
	_rshunt = fvalue;
	ph = param_find("INA219b_SHUNT");

	if (ph != PARAM_INVALID && param_get(ph, &fvalue) == PX4_OK) {
		_rshunt = fvalue;
	}

	ph = param_find("INA219b_CONFIG");
	int32_t value = INA219b_CONFIG;
	_config = (uint16_t)value;

	if (ph != PARAM_INVALID && param_get(ph, &value) == PX4_OK) {
		_config = (uint16_t)value;
	}

	_mode_trigged = false;

	_current_lsb = _max_current / DN_MAX;
	_power_lsb = 20 * _current_lsb;
}

INA219b::~INA219b()
{
	/* make sure we are truly inactive */
	stop();

	/* free perf counters */
	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int INA219b::read(uint8_t address)
{
	union {
		uint16_t reg;
		uint8_t  b[2] = {};
	} data;

	int ret = transfer(&address, 1, &data.b[0], sizeof(data.b));

	if (OK != ret) {
		perf_count(_comms_errors);
		PX4_DEBUG("i2c::transfer returned %d", ret);
		return -1;
	}

	return swap16(data.reg);
}

int INA219b::write(uint8_t address, uint16_t value)
{
	uint8_t data[3] = {address, ((uint8_t)((value & 0xff00) >> 8)), (uint8_t)(value & 0xff)};
	//return transfer(data, sizeof(data), nullptr, 0); //CTSHEN
	PX4_INFO("data = {0x%x, 0x%x, 0x%x}",data[0], data[1], data[2]); //CTSHEN
	int ret = transfer(data, sizeof(data), nullptr, 0); //CTSHEN
	PX4_INFO("ret = %d", ret); //CTSHEN
	return ret; //CTSHEN
}

int
INA219b::init()
{
	int ret = PX4_ERROR;

	/* do I2C init (and probe) first */
	if (I2C::init() != OK) {
		PX4_INFO("I2C init failed"); //CTSHEN
		return ret;
	}

	write(INA219b_REG_CONFIGURATION, INA219b_RST);

	_cal = INA219b_CONST / (_current_lsb * INA219b_SHUNT);
	PX4_INFO("_cal = %d ", _cal); //CTSHEN

	if (write(INA219b_REG_CALIBRATION, _cal) < 0) {
		PX4_INFO(" Write _cal to REG_CALIBRATION failed!"); //CTSHEN
		return -3;
	}

	// If we run in continuous mode then start it here

	if (!_mode_trigged) {
		ret = write(INA219b_REG_CONFIGURATION, _config);
	}
	ret = PX4_OK;

	set_device_address(INA219b_BASEADDR);	/* set I2c Address */

	start();
	_sensor_ok = true;

	PX4_INFO("ret = %d",ret); //CTSHEN
	return ret;
}

int
INA219b::probe()
{
	/*PX4_INFO("The probe is here!");
	int value = read(INA219b_MFG_ID);

	if (value < 0) {
		perf_count(_comms_errors);
	}

	if (value != INA219b_MFG_ID_TI) {
		PX4_DEBUG("probe mfgid %d", value);
		return -1;
	}

	value = read(INA219b_MFG_DIEID);

	if (value < 0) {
		perf_count(_comms_errors);
	}

	if (value != INA219b_MFG_DIE) {
		PX4_DEBUG("probe die id %d", value);
		return -1;
	}*/

	return OK;
}

int
INA219b::measure()
{
	int ret = OK;

	if (_mode_trigged) {
		ret = write(INA219b_REG_CONFIGURATION, _config);

		if (ret < 0) {
			perf_count(_comms_errors);
			PX4_DEBUG("i2c::transfer returned %d", ret);
		}
	}

	return ret;
}

int
INA219b::collect()
{
	int ret = -EIO;

	/* read from the sensor */
	perf_begin(_sample_perf);

	ret = _bus_volatage = read(INA219b_REG_BUSVOLTAGE);

	if (true) {  // (_bus_volatage>>3) >= 0
		ret = _power = read(INA219b_REG_POWER);
		//PX4_INFO("BV!!");

		if (true) { //_power >= 0
			ret = _current = read(INA219b_REG_CURRENT);

			if (true) { //_current >= 0
				ret = _shunt = read(INA219b_REG_SHUNTVOLTAGE);

				if (true) { // _shunt >= 0

					struct power_monitor_s report;
					report.timestamp = hrt_absolute_time();
					report.voltage_v = (float) ((uint16_t)_bus_volatage>>3 | 0x0000) * INA219b_VSCALE;
					report.current_a = (float) _current * _current_lsb;
					report.power_w   = (float) _power * _power_lsb;
#if defined(INA219b_RAW)
					report.rconf  = read(INA219b_REG_CONFIGURATION);
					report.rsv    = read(INA219b_REG_SHUNTVOLTAGE);
					report.rbv    = read(INA219b_REG_BUSVOLTAGE);
					report.rp     = read(INA219b_REG_POWER);
					report.rc     = read(INA219b_REG_CURRENT);
					report.rcal   = read(INA219b_REG_CALIBRATION);


#endif

					/* publish it */
					int instance;
					orb_publish_auto(ORB_ID(power_solar_in), &_power_monitor_topic, &report, &instance, ORB_PRIO_DEFAULT);

					ret = OK;
					perf_end(_sample_perf);
					return ret;
				}
			}
		}
	}

	PX4_DEBUG("error reading from sensor: %d", ret);
	perf_count(_comms_errors);
	perf_end(_sample_perf);
	return ret;
}

void
INA219b::start()
{
	ScheduleClear();

	/* reset the report ring and state machine */
	_collect_phase = false;

	_measure_interval = INA219b_CONVERSION_INTERVAL;

	/* schedule a cycle to start things */
	ScheduleDelayed(5);
}

void
INA219b::stop()
{
	ScheduleClear();
}

void
INA219b::Run()
{
	if (_collect_phase) {

		/* perform collection */
		if (OK != collect()) {
			PX4_DEBUG("collection error");
			/* if error restart the measurement state machine */
			start();
			return;
		}

		/* next phase is measurement */
		_collect_phase = !_mode_trigged;

		if (_measure_interval > INA219b_CONVERSION_INTERVAL) {

			/* schedule a fresh cycle call when we are ready to measure again */
			ScheduleDelayed(_measure_interval - INA219b_CONVERSION_INTERVAL);
			return;
		}
	}

	/* Measurement  phase */

	/* Perform measurement */
	if (OK != measure()) {
		PX4_DEBUG("measure error ina219b");
	}

	/* next phase is collection */
	_collect_phase = true;

	/* schedule a fresh cycle call when the measurement is done */
	ScheduleDelayed(INA219b_CONVERSION_INTERVAL);
}

void
INA219b::print_info()
{
	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	printf("poll interval:  %u \n", _measure_interval);
}

/**
 * Local functions in support of the shell command.
 */
namespace ina219b
{

INA219b	*g_dev;

int 	start();
int 	start_bus(int i2c_bus);
int 	stop();
int 	info();

/**
 *
 * Attempt to start driver on all available I2C busses.
 *
 * This function will return as soon as the first sensor
 * is detected on one of the available busses or if no
 * sensors are detected.
 *
 */
int
start()
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
		//PX4_INFO("b comes to here!"); //CTSHEN
		PX4_INFO("i2c_bus_options[%d] = %d", i, i2c_bus_options[i]);
		if (start_bus(i2c_bus_options[i]) == PX4_OK) {
			PX4_INFO("check point 2"); //CTSHEN
			return PX4_OK;
		}
	}
	PX4_ERR("NOTHING HAPPENED"); //CTSHEN
	return PX4_ERROR;
}

/**
 * Start the driver on a specific bus.
 *
 * This function only returns if the sensor is up and running
 * or could not be detected successfully.
 */
int
start_bus(int i2c_bus)
{
	if (g_dev != nullptr) {
		PX4_ERR("already started");
		return PX4_ERROR;
	}

	/* create the driver */
	g_dev = new INA219b(i2c_bus, INA219b_BASEADDR);
	PX4_INFO("new driver created! %p", g_dev); //CTSHEN

	if (g_dev == nullptr) {
		PX4_INFO("driver create fail 1"); // CTSHEN
		goto fail;
	}

	if (OK != g_dev->init()) {
		PX4_INFO("driver create failed 2"); //CTSHEN
		goto fail;
	}

	return PX4_OK;

fail:

	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;
	}

	return PX4_ERROR;
}

/**
 * Stop the driver
 */
int
stop()
{
	if (g_dev != nullptr) {
		delete g_dev;
		g_dev = nullptr;

	} else {
		PX4_ERR("driver not running");
		return PX4_ERROR;
	}

	return PX4_OK;
}

/**
 * Print a little info about the driver.
 */
int
info()
{
	if (g_dev == nullptr) {
		PX4_ERR("driver poll restart failed");
		return PX4_ERROR;
	}

	printf("state @ %p\n", g_dev);
	g_dev->print_info();

	return PX4_OK;
}

} /* namespace */


static void
ina2192b_usage()
{
	PX4_INFO("usage: ina219b command [options]");
	PX4_INFO("options:");
	PX4_INFO("\t-b --bus i2cbus (%d)", INA219b_BUS_DEFAULT);
	PX4_INFO("\t-a --all");
	PX4_INFO("command:");
	PX4_INFO("\tstart|stop|test|info");
}

int
ina219b_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	bool start_all = false;

	int i2c_bus = INA219b_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {

		case 'b':
			PX4_INFO("case b"); // CTSHEN
			i2c_bus = atoi(myoptarg);
			PX4_INFO("i2c_bus = %d", i2c_bus); //CTSHEN
			break;

		case 'a':
			PX4_INFO("case a"); //CTSHEN
			start_all = true;
			break;

		default:
			PX4_WARN("Unknown option!");
			goto out_error;
		}
	}

	if (myoptind >= argc) {
		goto out_error;
	}

	/*
	 * Start/load the driver.
	 */
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return ina219b::start();

		} else {
			return ina219b::start_bus(i2c_bus);
		}
	}

	/*
	 * Stop the driver
	 */
	if (!strcmp(argv[myoptind], "stop")) {
		return ina219b::stop();
	}

	/*
	 * Print driver information.
	 */
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return ina219b::info();
	}

out_error:
	ina2192b_usage();
	return PX4_ERROR;
}
