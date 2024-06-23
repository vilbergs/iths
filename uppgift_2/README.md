# Uppgift 2

**Note:** Sometimes the screen gets garbled on start/reset, hitting the reset button on the mcu will eventually make it start in a sane state. I could not figure out how to reliably reset the screen.

## PIN LAYOUT

### I2C

Both components are connected on the same bus and configured in `sdkconfig.defaults` as:

```
CONFIG_I2C_MASTER_SCL=19
CONFIG_I2C_MASTER_SDA=18
```

### Buttons

**Pull config:** Internal pull-up configuration, i.e. buttons only need to be connected to ground and GPIO pins

- Button 1 (back): `GPIO_NUM_0`
- Button 2 (next): `GPIO_NUM_0`
