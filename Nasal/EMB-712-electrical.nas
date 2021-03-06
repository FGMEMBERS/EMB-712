##
# pa28-161 electrical system.
# Edit of pa24-250 electrical system file using information from 
# 
##

# Initialize internal values
#

var battery = nil;
var alternator = nil;

var last_time = 0.0;

var vcutoff = 8.0;
var vbus_volts = 0.0;
var ebus1_volts = 0.0;
var ebus2_volts = 0.0;
var ammeter_lowpass = aircraft.lowpass.new(0.5);

##
# Initialize the electrical system
##

init_electrical = func {
    battery = BatteryClass.new();
    alternator = AlternatorClass.new();

    setprop("controls/electric/engine/generator", 0);
    setprop("controls/engines/engine[0]/fuel-pump",0);
    setprop("controls/engines/engine[0]/primer-pump",0);
    setprop("controls/switches/oat-switch", 0);
    setprop("controls/switches/nav-lights", 0);    
    setprop("controls/switches/panel-lights", 0);    
    setprop("controls/switches/panel-lights-factor", 0);    
    setprop("controls/switches/landing-light", 0);
    setprop("controls/switches/flashing-beacon",0);
    setprop("instrumentation/turn-indicator/serviceable",1);
    setprop("controls/anti-ice/pitot-heat", 0);
    setprop("controls/switches/starter", 0);
    setprop("controls/switches/strobe-lights", 0);
#    setprop("controls/switches/master-avionics", 0);
    setprop("controls/switches/map-lights",0);
    setprop("controls/switches/cabin-lights",0);
    setprop("systems/electrical/outputs/starter[0]", 0.0);
    setprop("systems/electrical/amps", 0.0);
    setprop("systems/electrical/volts", 0.0);
    setprop("systems/electrical/outputs/cabin-lights", 0.0);
    setprop("systems/electrical/outputs/instr-ignition-switch", 0.0);
    setprop("systems/electrical/outputs/fuel-pump", 0.0);
    setprop("systems/electrical/outputs/landing-light", 0.0);
    setprop("controls/lighting/landing-lights", 0);
    setprop("controls/lighting/beacon", 0);
    setprop("systems/electrical/outputs/fin-strobe-lights", 0.0 );
    setprop("systems/electrical/outputs/wing-strobe-lights", 0.0 );
    setprop("controls/lighting/fin-strobe-lights", 0);
    setprop("controls/lighting/panel-norm", 0.0);
    setprop("systems/electrical/outputs/turn-coordinator", 0.0);
    setprop("systems/electrical/outputs/nav-lights", 0.0);      
    setprop("controls/lighting/nav-lights", 0);
    setprop("systems/electrical/outputs/instrument-lights", 0.0);      
    setprop("systems/electrical/outputs/pitot-heat", 0.0);
    setprop("systems/electrical/outputs/nav[0]", 0.0);
    setprop("systems/electrical/outputs/comm[0]", 0.0);
    setprop("systems/electrical/outputs/dme", 0.0);
    setprop("systems/electrical/outputs/nav[1]", 0.0);
    setprop("systems/electrical/outputs/comm[1]", 0.0);
    setprop("systems/electrical/outputs/transponder", 0.0);
    setprop("systems/electrical/outputs/autopilot", 0.0);
    setprop("systems/electrical/outputs/adf", 0.0);
  
    setprop("engines/engine[0]/fuel-flow-gph", 0.0);
    setprop("surface-positions/flap-pos-norm",0.0);
    setprop("instrumentation/airspeed-indicator/indicated-speed-kt", 0.0);
    setprop("accelerations/pilot-g", 1.0);
    print("Nasal Electrical System Initialized");  

    # Request that the update fuction be called next frame
    settimer(update_electrical, 0);
}

BatteryClass = {};

BatteryClass.new = func {
    obj = { parents : [BatteryClass],
            ideal_volts : 12.0,
            ideal_amps : 35.0,
            amp_hours : 12.75,
            charge_percent : 1.0,
            charge_amps : 7.0 };
    return obj;
}


BatteryClass.apply_load = func( amps, dt ) {
    var amphrs_used = amps * dt / 3600.0;
    var percent_used = amphrs_used / me.amp_hours;
    me.charge_percent -= percent_used;
    if ( me.charge_percent < 0.0 ) {
        me.charge_percent = 0.0;
    } elsif ( me.charge_percent > 1.0 ) {
        me.charge_percent = 1.0;
    }
    # print( "battery percent = ", me.charge_percent);
    return me.amp_hours * me.charge_percent;
}


BatteryClass.get_output_volts = func {
    var x = 1.0 - me.charge_percent;
    var tmp = -(3.0 * x - 1.0);
    var factor = (tmp*tmp*tmp*tmp*tmp + 32) / 32;
    return me.ideal_volts * factor;
}


BatteryClass.get_output_amps = func {
    var x = 1.0 - me.charge_percent;
    var tmp = -(3.0 * x - 1.0);
    var factor = (tmp*tmp*tmp*tmp*tmp + 32) / 32;
    return me.ideal_amps * factor;
}


AlternatorClass = {};

AlternatorClass.new = func {
    obj = { parents : [AlternatorClass],
            rpm_source : "engines/engine[0]/rpm",
            rpm_threshold : 600.0,
            ideal_volts : 14.0,
            ideal_amps : 50.0 };
    return obj;
}


AlternatorClass.apply_load = func( amps, dt ) {
    # Scale alternator output for rpms < 600.  For rpms >= 600
    # give full output.  This is just a WAG, and probably not how
    # it really works but I'm keeping things "simple" to start.
    var rpm = getprop( me.rpm_source );
    var factor = rpm / me.rpm_threshold;
    if ( factor > 1.0 ) {
        factor = 1.0;
    }
    # print( "alternator amps = ", me.ideal_amps * factor );
    var available_amps = me.ideal_amps * factor;
    return available_amps - amps;
}


AlternatorClass.get_output_volts = func {
    # scale alternator output for rpms < 600.  For rpms >= 600
    # give full output.  This is just a WAG, and probably not how
    # it really works but I'm keeping things "simple" to start.
    var rpm = getprop( me.rpm_source );
    var factor = rpm / me.rpm_threshold;
    if ( factor > 1.0 ) {
        factor = 1.0;
    }
    # print( "alternator volts = ", me.ideal_volts * factor );
    return me.ideal_volts * factor;
}


AlternatorClass.get_output_amps = func {
    # scale alternator output for rpms < 600.  For rpms >= 600
    # give full output.  This is just a WAG, and probably not how
    # it really works but I'm keeping things "simple" to start.
    var rpm = getprop( me.rpm_source );
    var factor = rpm / me.rpm_threshold;
    if ( factor > 1.0 ) {
        factor = 1.0;
    }
    # print( "alternator amps = ", ideal_amps * factor );
    return me.ideal_amps * factor;
}


update_electrical = func {
    var time = getprop("sim/time/elapsed-sec");
    var dt = time - last_time;
    last_time = time;

    update_virtual_bus( dt );

    # Request that the update fuction be called again next frame
    settimer(update_electrical, 0);
}



update_virtual_bus = func( dt ) {
    var battery_volts = battery.get_output_volts();
    var alternator_volts = alternator.get_output_volts();
    var external_volts = 0.0;
    var load = 0.0;

    # switch state
    var master_bat = getprop("controls/electric/battery-switch");
    if ( master_bat ) {
        setprop("controls/electric/engine/generator",1);
    }

    var master_alt = getprop("controls/electric/alternator-switch");;

    # determine power source
    var bus_volts = 0.0;
    var power_source = nil;
    if ( master_bat ) {
        bus_volts = battery_volts;
        power_source = "battery";
    }

    if ( master_alt and (alternator_volts > bus_volts) ) {
        bus_volts = alternator_volts;
        power_source = "alternator";
    }

    if ( external_volts > bus_volts ) {
        bus_volts = external_volts;
        power_source = "external";
    }
    # print( "virtual bus volts = ", bus_volts );

    # starter motor
    starter_switch = getprop("controls/switches/starter");
    starter_volts = 0.0;
    if ( starter_switch ) {
        starter_volts = bus_volts;
        load += 12;
    }
    setprop("systems/electrical/outputs/starter[0]", starter_volts);
    if (starter_volts > vcutoff) {
    setprop("controls/engines/engine[0]/starter",1);
    setprop("controls/engines/engine[0]/magnetos",3);
    } else {
    setprop("controls/engines/engine[0]/starter",0);
    }

    # bus network (1. these must be called in the right order, 2. the
    # bus routine itself determines where it draws power from.)
    load += electrical_bus_1();
    load += electrical_bus_2();
    load += cross_feed_bus();
    load += avionics_bus_1();
    load += avionics_bus_2();

    # system loads and ammeter gauge
    ammeter = 0.0;
    if ( bus_volts > 1.0 ) {
        # normal load
        load += 15.0;

        # ammeter gauge
        if ( power_source == "battery" ) {
            ammeter = -load;
        } else {
            ammeter = battery.charge_amps;
        }
    }

    # charge/discharge the battery
    if ( power_source == "battery" ) {
        battery.apply_load( load, dt );
    } elsif ( bus_volts > battery_volts ) {
        battery.apply_load( -battery.charge_amps, dt );
    }

    # outputs

    setprop("systems/electrical/amps", ammeter_lowpass.filter(ammeter));
    setprop("systems/electrical/volts", bus_volts);
    vbus_volts = bus_volts;

    return load;
}

electrical_bus_1 = func() {
    # we are fed from the "virtual" bus
    bus_volts = vbus_volts;
    load = 0.0;
    
    # Cabin Lights Power
    if ( getprop("controls/switches/cabin-lights") ) {
        setprop("systems/electrical/outputs/cabin-lights", bus_volts);
    } else {
        setprop("systems/electrical/outputs/cabin-lights", 0.0);
    }
    if ( getprop("systems/electrical/outputs/cabin-lights") > vcutoff)
    {
        setprop("controls/lighting/cabin-lights", 1);
        load += 0.2;
    } else {
        setprop("controls/lighting/cabin-lights", 0);
    }

   # Map Lights Power
    if ( getprop("controls/switches/map-lights") ) {
        setprop("systems/electrical/outputs/map-lights", bus_volts);
    } else {
        setprop("systems/electrical/outputs/map-lights", 0.0);
    }
    if ( getprop("systems/electrical/outputs/map-lights") > vcutoff)
    {
        setprop("controls/lighting/map-lights", 1);
        load += 0.2;
    } else {
        setprop("controls/lighting/map-lights", 0);
    }
    # Instrument Power
    setprop("systems/electrical/outputs/instr-ignition-switch", bus_volts);
    if (bus_volts > vcutoff) {
    load += 0.3;
    }

    # Fuel Pump Power
    if ( getprop("controls/engines/engine[0]/fuel-pump") ) {
        setprop("systems/electrical/outputs/fuel-pump", bus_volts);
        load += 0.1;
    } else {
        setprop("systems/electrical/outputs/fuel-pump", 0.0);
    }

    # Landing Light Power
    if ( getprop("controls/switches/landing-light") ) {
        setprop("systems/electrical/outputs/landing-light", bus_volts);
    } else {
        setprop("systems/electrical/outputs/landing-light", 0.0 );
    }
    if ( getprop("systems/electrical/outputs/landing-light") > vcutoff)
    {
        setprop("controls/lighting/landing-lights", 1);
        load += 3.2;
    } else {
        setprop("controls/lighting/landing-lights", 0);
    }

    # Fin Strobe Power
    if ( getprop("controls/switches/flashing-beacon") ) {
        setprop("systems/electrical/outputs/flashing-beacon", bus_volts);
    } else {
        setprop("systems/electrical/outputs/flashing-beacon", 0.0 );
    }
    if ( getprop("systems/electrical/outputs/flashing-beacon") > vcutoff )
    {
        setprop("controls/lighting/beacon", 1);
        load += 1.2;
    } else {
        setprop("controls/lighting/beacon", 0);
    }

    # Wing Strobe Power
    if ( getprop("controls/switches/strobe-lights") ) {
        setprop("systems/electrical/outputs/strobe-lights", bus_volts);
    } else {
        setprop("systems/electrical/outputs/strobe-lights", 0.0 );
    }
    if ( getprop("systems/electrical/outputs/strobe-lights") > vcutoff ) 
    {
        setprop("controls/lighting/strobe", 1);
        load += 1.2;
    } else {
        setprop("controls/lighting/strobe", 0);
    }

    # register bus voltage
    ebus1_volts = bus_volts;

    # return cumulative load
    return load;
}


electrical_bus_2 = func() {
    # we are fed from the "virtual" bus
    bus_volts = vbus_volts;
    load = 0.0;

    # Turn Coordinator Power
    setprop("systems/electrical/outputs/turn-coordinator", bus_volts);
  
    # Nav Lights Power
    if ( getprop("controls/switches/nav-lights" ) ) {
        setprop("systems/electrical/outputs/nav-lights", bus_volts);
    } else {
        setprop("systems/electrical/outputs/nav-lights", 0.0);      
    }

    if ( getprop("systems/electrical/outputs/nav-lights") > vcutoff ) 
    {
        setprop("controls/lighting/nav-lights", 1);
        load += 1.4;
    } else {
        setprop("controls/lighting/nav-lights", 0);
    }

    # Instrument Lights Power controlled by panel-lights switch on pa28
    factor = getprop("controls/switches/panel-lights-factor");
    if ( factor > 0.005 )  {
    setprop("controls/switches/panel-lights", 1);
    setprop("systems/electrical/outputs/instrument-lights", bus_volts);
# Normalize factor by 1/14 = 0.071428571 for max bus_volts
    setprop("sim/model/material/instruments/factor", bus_volts * 0.071428571 * ( 1.0 - factor));
    setprop("systems/electrical/outputs/instrument-lights-norm", bus_volts * 0.071428571 * (1.0 - factor));
    } else {
    setprop("systems/electrical/outputs/instrument-lights", 0.0);
    setprop("sim/model/material/instruments/factor", 0.0);
    setprop("controls/switches/panel-lights", 0.0);
    setprop("systems/electrical/outputs/instrument-lights-norm", 0.0);
    }




    # Pitot Heat Power
    if ( getprop("controls/anti-ice/pitot-heat" ) ) {
        setprop("systems/electrical/outputs/pitot-heat", bus_volts);
        if (bus_volts > vcutoff) {load += 12.8; }
    } else {
        setprop("systems/electrical/outputs/pitot-heat", 0.0);
    }
  
    # register bus voltage
    ebus2_volts = bus_volts;

    # return cumulative load
    return load;
}


cross_feed_bus = func() {
    # we are fed from either of the electrical bus 1 or 2
    if ( ebus1_volts > ebus2_volts ) {
        bus_volts = ebus1_volts;
    } else {
        bus_volts = ebus2_volts;
    }

    load = 0.0;

    setprop("systems/electrical/outputs/annunciators", bus_volts);

    # return cumulative load
    return load;
}


avionics_bus_1 = func() {

    bus_volts = ebus1_volts;
    load = 0.0;

    # Nav 1 Power
    setprop("systems/electrical/outputs/nav[0]", bus_volts);
        if (bus_volts > vcutoff) {load += 0.8; }
    # Com 1 Power
    setprop("systems/electrical/outputs/comm[0]", bus_volts);
        if (bus_volts > vcutoff) {load += 0.8; }
    # DME
    setprop("systems/electrical/outputs/dme", bus_volts);
        if (bus_volts > vcutoff) {load += 0.6; }
    # return cumulative load
    return load;
}


avionics_bus_2 = func() {

    bus_volts = ebus2_volts;
    load = 0.0;

    # Nav 2 Power
    setprop("systems/electrical/outputs/nav[1]", bus_volts);
        if (bus_volts > vcutoff) {load += 0.8; }
    # Com 2 Power
    setprop("systems/electrical/outputs/comm[1]", bus_volts);
        if (bus_volts > vcutoff) {load += 0.8; }


    # Transponder Power
    setprop("systems/electrical/outputs/transponder", bus_volts);
        if (bus_volts > vcutoff) {load += 0.6; }

    # Autopilot Power
    setprop("systems/electrical/outputs/autopilot", bus_volts);
        if (bus_volts > vcutoff) {load += 0.8; }

    # ADF Power
    setprop("systems/electrical/outputs/adf", bus_volts);
        if (bus_volts > vcutoff) {load += 0.8; }

    # return cumulative load
    return load;
}


# Setup listener call to initialize the electrical system once the fdm is initialized
# 
setlistener("sim/signals/fdm-initialized", init_electrical);  


