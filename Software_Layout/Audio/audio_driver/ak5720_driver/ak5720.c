/*
 * ALSA SoC driver for
 *    Asahi Kasei AK5720 Single-ended 24-Bit ADC
 *
 * (c) 2023 Your Name
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published by
 * the Free Software Foundation.
 */

 #include <linux/module.h>
 #include <linux/slab.h>
 #include <linux/of.h>
 #include <linux/of_gpio.h>
 #include <linux/of_device.h>
 #include <linux/regulator/consumer.h>
 #include <sound/soc.h>
 #include <sound/soc-dai.h>    /* For DAI definitions */
 #include <sound/pcm.h>
 #include <sound/initval.h>
 
 /* Function prototypes */
 static int ak5720_component_probe(struct snd_soc_component *component);
 static void ak5720_component_remove(struct snd_soc_component *component);
 #ifdef CONFIG_PM
 static int ak5720_component_suspend(struct snd_soc_component *component);
 static int ak5720_component_resume(struct snd_soc_component *component);
 #endif
 static int ak5720_set_dai_fmt(struct snd_soc_dai *dai, unsigned int format);
 static int ak5720_hw_params(struct snd_pcm_substream *substream,
							 struct snd_pcm_hw_params *params,
							 struct snd_soc_dai *dai);
 static int ak5720_hw_free(struct snd_pcm_substream *substream,
						   struct snd_soc_dai *dai);
 
 /* Supply names used by the regulator API */
 static const char * const supply_names[] = {
	 "va", "vd"
 };
 
 struct ak5720_priv {
	 int reset_gpio;
	 struct regulator_bulk_data supplies[ARRAY_SIZE(supply_names)];
 };
 
 /* DAPM widgets and routes */
 static const struct snd_soc_dapm_widget ak5720_dapm_widgets[] = {
	 SND_SOC_DAPM_INPUT("LIN"),
	 SND_SOC_DAPM_INPUT("RIN"),
 };
 
 static const struct snd_soc_dapm_route ak5720_dapm_routes[] = {
	 { "Capture", NULL, "LIN" },
	 { "Capture", NULL, "RIN" },
 };
 
 /* Component (codec) callbacks */
 static int ak5720_component_probe(struct snd_soc_component *component)
 {
	 struct ak5720_priv *priv = snd_soc_component_get_drvdata(component);
	 dev_info(component->dev, "AK5720 codec probed\n");
	 return regulator_bulk_enable(ARRAY_SIZE(priv->supplies), priv->supplies);
 }
 
 static void ak5720_component_remove(struct snd_soc_component *component)
 {
	 struct ak5720_priv *priv = snd_soc_component_get_drvdata(component);
	 regulator_bulk_disable(ARRAY_SIZE(priv->supplies), priv->supplies);
 }
 
 #ifdef CONFIG_PM
 static int ak5720_component_suspend(struct snd_soc_component *component)
 {
	 struct ak5720_priv *priv = snd_soc_component_get_drvdata(component);
	 regulator_bulk_disable(ARRAY_SIZE(priv->supplies), priv->supplies);
	 return 0;
 }
 
 static int ak5720_component_resume(struct snd_soc_component *component)
 {
	 struct ak5720_priv *priv = snd_soc_component_get_drvdata(component);
	 return regulator_bulk_enable(ARRAY_SIZE(priv->supplies), priv->supplies);
 }
 #else
 #define ak5720_component_suspend NULL
 #define ak5720_component_resume  NULL
 #endif
 
 static const struct snd_soc_component_driver soc_component_ak5720 = {
	 .probe            = ak5720_component_probe,
	 .remove           = ak5720_component_remove,
	 .suspend          = ak5720_component_suspend,
	 .resume           = ak5720_component_resume,
	 .dapm_widgets     = ak5720_dapm_widgets,
	 .num_dapm_widgets = ARRAY_SIZE(ak5720_dapm_widgets),
	 .dapm_routes      = ak5720_dapm_routes,
	 .num_dapm_routes  = ARRAY_SIZE(ak5720_dapm_routes),
 };
 
 static int ak5720_set_dai_fmt(struct snd_soc_dai *dai, unsigned int format)
 {
	 /* Use dai->component instead of snd_soc_dai_get_component() */
	 struct snd_soc_component *component = dai->component;
	 format &= SND_SOC_DAIFMT_FORMAT_MASK;
	 if (format != SND_SOC_DAIFMT_I2S &&
		 format != SND_SOC_DAIFMT_LEFT_J) {
		 dev_err(component->dev, "Invalid DAI format\n");
		 return -EINVAL;
	 }
	 return 0;
 }
 
 static int ak5720_hw_params(struct snd_pcm_substream *substream,
				 struct snd_pcm_hw_params *params,
				 struct snd_soc_dai *dai)
 {
	 /* Use dai->component */
	 struct snd_soc_component *component = dai->component;
	 struct ak5720_priv *priv = snd_soc_component_get_drvdata(component);
 
	 /* All mode settings are fixed externally. Just bring the device out of reset. */
	 if (gpio_is_valid(priv->reset_gpio)) {
		 gpio_set_value(priv->reset_gpio, 1);  /* Bring device out of reset */
		 msleep(50);  /* Wait ~50ms for initialization */
	 }
	 return 0;
 }
 
 static int ak5720_hw_free(struct snd_pcm_substream *substream,
			   struct snd_soc_dai *dai)
 {
	 struct snd_soc_component *component = dai->component;
	 struct ak5720_priv *priv = snd_soc_component_get_drvdata(component);
	 if (gpio_is_valid(priv->reset_gpio))
		 gpio_set_value(priv->reset_gpio, 0);  /* Optionally assert reset on free */
	 return 0;
 }
 
 static const struct snd_soc_dai_ops ak5720_dai_ops = {
	 .set_fmt   = ak5720_set_dai_fmt,
	 .hw_params = ak5720_hw_params,
	 .hw_free   = ak5720_hw_free,
 };
 
 static struct snd_soc_dai_driver ak5720_dai = {
	 .name       = "ak5720-hifi",
	 .capture    = {
		 .stream_name    = "Capture",
		 .channels_min   = 1,
		 .channels_max   = 2,
		 /* AK5720 supports 8kHz to 96kHz */
		 .rates          = SNDRV_PCM_RATE_8000_96000,
		 .formats        = SNDRV_PCM_FMTBIT_S16_LE |
				   SNDRV_PCM_FMTBIT_S24_LE |
				   SNDRV_PCM_FMTBIT_S24_3LE,
	 },
	 .ops        = &ak5720_dai_ops,
 };
 
 #ifdef CONFIG_OF
 static const struct of_device_id ak5720_dt_ids[] = {
	 { .compatible = "akm,ak5720", },
	 { }
 };
 MODULE_DEVICE_TABLE(of, ak5720_dt_ids);
 #endif
 
 static int ak5720_probe(struct platform_device *pdev)
 {
	 struct device *dev = &pdev->dev;
	 struct ak5720_priv *priv;
	 int ret, i;
 
	 dev_info(dev, "Probing AK5720 driver...\n");  // Add this line
	 priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	 if (!priv) {
		 dev_err(dev, "Failed to allocate memory for priv\n");
		 return -ENOMEM;
	 }
	 priv->reset_gpio = -EINVAL;
	 dev_set_drvdata(dev, priv);
 
	 for (i = 0; i < ARRAY_SIZE(supply_names); i++) {
		 priv->supplies[i].supply = supply_names[i];
		 dev_dbg(dev, "Requesting regulator: %s\n", supply_names[i]);  // Debug line here
	 }
	 ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(priv->supplies), priv->supplies);
	 if (ret < 0) {
		 dev_err(dev, "Failed to get regulators: %d\n", ret);
		 return ret;
	 }
 
	 // GPIO initialization and reset
	 if (of_match_device(of_match_ptr(ak5720_dt_ids), dev))
		 priv->reset_gpio = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
 
	 if (gpio_is_valid(priv->reset_gpio)) {
		 dev_dbg(dev, "Reset GPIO valid, configuring...\n");  // Debug line here
		 if (devm_gpio_request_one(dev, priv->reset_gpio, GPIOF_OUT_INIT_LOW, "AK5720 Reset")) {
			 dev_err(dev, "Failed to request reset GPIO\n");
			 priv->reset_gpio = -EINVAL;
		 }
	 } else {
		 dev_err(dev, "Invalid reset GPIO\n");
	 }
 
	 ret = devm_snd_soc_register_component(dev, &soc_component_ak5720, &ak5720_dai, 1);
	 if (ret) {
		 dev_err(dev, "Failed to register AK5720 codec: %d\n", ret);
		 return ret;
	 }
 
	 dev_info(dev, "AK5720 codec successfully probed.\n");  // Debug line here
	 return 0;
 }
 
 static int ak5720_remove(struct platform_device *pdev)
 {
	 snd_soc_unregister_component(&pdev->dev);
	 return 0;
 }
 
 static struct platform_driver ak5720_driver = {
	 .probe      = ak5720_probe,
	 .remove     = ak5720_remove,
	 .driver     = {
		 .name           = "ak5720",
		 .of_match_table = of_match_ptr(ak5720_dt_ids),
	 },
 };
 
 module_platform_driver(ak5720_driver);
 
 MODULE_DESCRIPTION("ASoC driver for AK5720 ADC");
 MODULE_AUTHOR("Catherine and Carina");
 MODULE_LICENSE("GPL");