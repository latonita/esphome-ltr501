Preliminary ver for LTR-501ALS (ALS + PS) and LTR-301ALS (ALS only)


```
external_components:
   - source: github://latonita/esphome-ltr501
     components: [ ltr501 ]

i2c:
#...

sensor:
  - platform: ltr501
    address: 0x23
    update_interval: 10s
    type: ALS_PS
    gain: 1x #192x    
    integration_time: 100ms  #100ms - for both gains, 50ms - 1x only, 200ms and 400ms - 192x only
    ps_gain: 1x
    ps_cooldown: 5s
    # proximity_cooldown: 3 s
    # proximity_high_threshold: 590
    # proximity_low_threshold: 10
    # on_high_threshold:
    #   then:
    #     - logger.log: "Proximity high!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
    # on_low_threshold:
    #   then:
    #     - logger.log: "Proximity low high ------------------------------------------------------------"

    infrared_counts:
      name: "CH1 Infrared counts"
    full_spectrum_counts:
      name: "CH0 Full spectrum counts"
    ambient_light:
      name: "Ambient Light"

    ps_counts:
      name: "Proximity counts" 

```