/*
 * Driver for hifibunny-codec
 *
 * Author: Satoru Kawase
 *      Copyright 2018
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */


#include <linux/init.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/i2c.h>
#include <sound/soc.h>
#include <sound/pcm_params.h>
#include <sound/tlv.h>
//#include "hifibunny-codec.h"
//
#define VOLUME1		0x0F
#define VOLUME2		0x10
#define GENERAL_SET	0x07
#define INPUT_CONFIG	0x01
#define DPLL		0x0C
#define M_MODE		0x0A
#define GPIO_config	0x08
static int hifibunny_codec_dac_mute(struct snd_soc_dai *dai, int mute);
/* hifibunny Q2M Codec Private Data */
struct hifibunny_codec_priv {
	struct regmap *regmap;
	unsigned int fmt;
};

/* hifibunny Q2M Default Register Value */
static const struct reg_default hifibunny_codec_reg_defaults[] = {
	{INPUT_CONFIG,0x8c},
	{GENERAL_SET,0x87},
	{GPIO_config,0x10},
	{M_MODE,0x00},
	{DPLL,0x5A},
	{0x0D,0x00},
	{VOLUME1,0xFF},
	{VOLUME2,0xFF},
	{0x16,0x00},//Lch 2nd THD fine
	{0x17,0x00},//Lch 2nd THD coarse
	{0x18, 0x00},//Lch 3nd THD fine
	{0x19,0x00},//Lch 3nd THD coarse
	{0x1A,0x00},
	{0x1B,0x00},
	{0x1C,0x00},
	{0x1D,0x00},
	{0x1E, 0x00},
	{0x22,0x00},//Rch 2nd THD fine
	{0x23, 0x00},//Rch 2nd THD coarse
	{0x24,0x00},//Rch 3rd THD fine
	{0x25,0x00},//Rch 3rd THD coarse
	{0x26,0x01},//L/R sep. THD comp.
};


static bool hifibunny_codec_writeable(struct device *dev, unsigned int reg)
{
	if(reg == 0)
		return false;
	else if(reg == 1)
		return true;
	else if(4 <= reg && reg <= 30)
		return true;
	else if(34 <= reg && reg <= 38)
		return true;
	else if(reg == 43)
		return true;
	else
		return false;
}

static bool hifibunny_codec_readable(struct device *dev, unsigned int reg)
{
	return true;
}

static bool hifibunny_codec_volatile(struct device *dev, unsigned int reg)
{
	return true;
}


/* Volume Scale */
static const DECLARE_TLV_DB_SCALE(volume_tlv, -12750, 50, 1);

/* Filter Type */
static const char * const fir_filter_type_texts[] = {
	"Fast Roll-Off",
	"Slow Roll-Off",
	"Minimum Phase",
};

static SOC_ENUM_SINGLE_DECL(hifibunny_fir_filter_type_enum,
				GENERAL_SET, 5, fir_filter_type_texts);


/* IIR Filter Select */
static const char * const iir_filter_texts[] = {
	"47kHz",
	"50kHz",
	"60kHz",
	"70kHz",
};

static SOC_ENUM_SINGLE_DECL(hifibunny_iir_filter_enum,
				GENERAL_SET, 2, iir_filter_texts);


/* I2S / SPDIF Select */
static const char * const iis_spdif_sel_texts[] = {
	"I2S",
	"SPDIF",
	"reserved",
	"DSD",
};

/*Enable or disable THD compensation*/
static SOC_ENUM_SINGLE_DECL(hifibunny_iis_spdif_sel_enum,
				INPUT_CONFIG, 0, iis_spdif_sel_texts);

static const char * const thd_enum_texts[] = 
{
	"Enable comp.",
	"Disable comp.",
};
static SOC_ENUM_SINGLE_DECL(hifibunny_thd_enum, 0x0D, 6, thd_enum_texts);

/*Compenate seperately or not*/
static const char * const thd_ctrl_texts[] = 
{
	"Sep. comp.",
	"Non Sep. comp.",
};
static SOC_ENUM_SINGLE_DECL(hifibunny_thd_ctrl_enum, 38, 0, thd_ctrl_texts);

static const char * const channel_mute_texts[] = 
{
	"Normal",
	"Mute Lch",
	"Mute Rch",
	"Mute both",
};
static SOC_ENUM_SINGLE_DECL(channel_mute_ctrl, GENERAL_SET, 0, channel_mute_texts);

/* Control */
static const struct snd_kcontrol_new hifibunny_codec_controls[] = {
SOC_DOUBLE_R_TLV("Digital Playback Volume", VOLUME1, VOLUME2,
		 0, 255, 1, volume_tlv),
SOC_ENUM("Ch Mute", channel_mute_ctrl),
SOC_ENUM("FIR Filter", hifibunny_fir_filter_type_enum),
SOC_ENUM("IIR Filter", hifibunny_iir_filter_enum),
SOC_ENUM("input", hifibunny_iis_spdif_sel_enum),
};


static const u32 hifibunny_codec_dai_rates_slave[] = {
	8000, 11025, 16000, 22050, 32000,
	44100, 48000, 64000, 88200, 96000, 176400, 192000, 352800, 384000
};

static const struct snd_pcm_hw_constraint_list constraints_slave = {
	.list  = hifibunny_codec_dai_rates_slave,
	.count = ARRAY_SIZE(hifibunny_codec_dai_rates_slave),
};

static int hifibunny_codec_dai_startup_slave(
		struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;
	int ret;

	ret = snd_pcm_hw_constraint_list(substream->runtime,
			0, SNDRV_PCM_HW_PARAM_RATE, &constraints_slave);
	if (ret != 0) {
		dev_err(codec->dev, "Failed to setup rates constraints: %d\n", ret);
	}

	return ret;
}

static int hifibunny_codec_dai_startup(
		struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	struct snd_soc_codec      * codec = dai->codec;
	struct hifibunny_codec_priv * hifibunny_codec
					= snd_soc_codec_get_drvdata(codec);
	hifibunny_codec_dac_mute(dai, 1);

	switch (hifibunny_codec->fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		return hifibunny_codec_dai_startup_slave(substream, dai);

	default:
		return (-EINVAL);
	}
}
static int hifibunny_codec_hw_params(
	struct snd_pcm_substream *substream, struct snd_pcm_hw_params *params,
	struct snd_soc_dai *dai)
{
	struct snd_soc_codec *codec = dai->codec;

	uint8_t iface = snd_soc_read(codec, INPUT_CONFIG) & 0x3f;
	switch (params_format(params)) {
		case SNDRV_PCM_FORMAT_S16_LE:
			iface |= 0x0;
			break;
		case SNDRV_PCM_FORMAT_S24_LE:
			iface |= 0x80;
			break;
		case SNDRV_PCM_FORMAT_S32_LE:
			iface |= 0x80;
			break;
		default:
			return -EINVAL;
	}
	snd_soc_write(codec, INPUT_CONFIG, iface);

	switch(params_rate(params))
	{
		case 11025:
		case 22050:
		case 44100:
		case 88200:
		case 176400:
		case 352800:
			snd_soc_write(codec, DPLL, 0x9A);
			break;
		case 8000:
		case 16000:
		case 32000:
		case 48000:
		case 64000:
		case 96000:
		case 192000:
		case 384000:
			snd_soc_write(codec, DPLL, 0x5A);
			break;
		default:
			snd_soc_write(codec, DPLL, 0x5A);
	}
	return 0;
}

static int hifibunny_codec_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct snd_soc_codec      *codec = dai->codec;
	struct hifibunny_codec_priv *hifibunny_codec
					= snd_soc_codec_get_drvdata(codec);

	/* interface format */
	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_I2S:
		break;

	case SND_SOC_DAIFMT_RIGHT_J:
	case SND_SOC_DAIFMT_LEFT_J:
	default:
		return (-EINVAL);
	}

	/* clock inversion */
	if ((fmt & SND_SOC_DAIFMT_INV_MASK) != SND_SOC_DAIFMT_NB_NF) {
		return (-EINVAL);
	}

	/* Set Audio Data Format */
	hifibunny_codec->fmt = fmt;

	return 0;
}

static int hifibunny_codec_dac_mute(struct snd_soc_dai *dai, int mute)
{
	uint8_t genSet = snd_soc_read(dai->codec, GENERAL_SET);
	if(mute)
	{
		snd_soc_write(dai->codec, GENERAL_SET, genSet | 0x03);
	}
	return 0;
}

static int hifibunny_codec_dac_unmute(struct snd_soc_dai *dai)
{
	uint8_t genSet = snd_soc_read(dai->codec, GENERAL_SET);
	snd_soc_write(dai->codec, GENERAL_SET, genSet & 0xFC);
	return 0;
}

static void hifibunny_codec_dai_shutdown(struct snd_pcm_substream * substream, struct snd_soc_dai *dai)
{
	hifibunny_codec_dac_mute(dai, 1);
}

static int hifibunny_codec_dai_prepare(struct snd_pcm_substream *substream, struct snd_soc_dai *dai)
{
	hifibunny_codec_dac_unmute(dai)
	return 0;
}

static int hifibunny_codec_dai_trigger(struct snd_pcm_substream *substream, int cmd, struct snd_soc_dai *dai)
{
	int ret = 0;
	switch(cmd)
	{
		case SNDRV_PCM_TRIGGER_START:
		case SNDRV_PCM_TRIGGER_RESUME:
		case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
			break;
		case SNDRV_PCM_TRIGGER_STOP:
		case SNDRV_PCM_TRIGGER_SUSPEND:
		case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
			hifibunny_codec_dac_mute(dai, 1)
			break;
		default:
			ret = -EINVAL;
			break;
	}
	return ret;
}
static const struct snd_soc_dai_ops hifibunny_codec_dai_ops = {
	.startup      = hifibunny_codec_dai_startup,
	.hw_params    = hifibunny_codec_hw_params,
	.set_fmt      = hifibunny_codec_set_fmt,
	.digital_mute = hifibunny_codec_dac_mute,
	.shutdown = hifibunny_codec_dai_shutdown,
	.prepare = hifibunny_codec_dai_prepare,
	.trigger = hifibunny_codec_dai_trigger,
};

static struct snd_soc_dai_driver hifibunny_codec_dai = {
	.name = "hifibunny-codec-dai",
	.playback = {
		.stream_name  = "Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min = 8000,
		.rate_max = 384000,
		.formats      = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &hifibunny_codec_dai_ops,
};

static struct snd_soc_codec_driver hifibunny_codec_codec_driver = {
	.component_driver = {
		.controls         = hifibunny_codec_controls,
		.num_controls     = ARRAY_SIZE(hifibunny_codec_controls),
	}
};


static const struct regmap_config hifibunny_codec_regmap = {
	.reg_bits         = 8,
	.val_bits         = 8,
	.max_register     = 74,

	.reg_defaults     = hifibunny_codec_reg_defaults,
	.num_reg_defaults = ARRAY_SIZE(hifibunny_codec_reg_defaults),

	.writeable_reg    = hifibunny_codec_writeable,
	.readable_reg     = hifibunny_codec_readable,
	.volatile_reg     = hifibunny_codec_volatile,

	.cache_type       = REGCACHE_RBTREE,
};


static int hifibunny_codec_probe(struct device *dev, struct regmap *regmap)
{
	struct hifibunny_codec_priv *hifibunny_codec;
	int ret;
	hifibunny_codec = devm_kzalloc(dev, sizeof(*hifibunny_codec), GFP_KERNEL);
	if (!hifibunny_codec) {
		dev_err(dev, "devm_kzalloc");
		return (-ENOMEM);
	}
	printk("Registering hifibunny-codec \n");
	hifibunny_codec->regmap = regmap;

	dev_set_drvdata(dev, hifibunny_codec);

	ret = snd_soc_register_codec(dev,
			&hifibunny_codec_codec_driver, &hifibunny_codec_dai, 1);
	if (ret != 0) {
		dev_err(dev, "Failed to register CODEC: %d\n", ret);
		return ret;
	}
	return 0;
}

static void hifibunny_codec_remove(struct device *dev)
{
	snd_soc_unregister_codec(dev);
}


static int hifibunny_codec_i2c_probe(
		struct i2c_client *i2c, const struct i2c_device_id *id)
{
	struct regmap *regmap;

	regmap = devm_regmap_init_i2c(i2c, &hifibunny_codec_regmap);
	if (IS_ERR(regmap)) {
		return PTR_ERR(regmap);
	}

	return hifibunny_codec_probe(&i2c->dev, regmap);
}

static int hifibunny_codec_i2c_remove(struct i2c_client *i2c)
{
	hifibunny_codec_remove(&i2c->dev);

	return 0;
}


static const struct i2c_device_id hifibunny_codec_i2c_id[] = {
	{ "hifibunny-codec", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hifibunny_codec_i2c_id);

static const struct of_device_id hifibunny_codec_of_match[] = {
	{ .compatible = "tuxiong,hifibunny-codec", },
	{ }
};
MODULE_DEVICE_TABLE(of, hifibunny_codec_of_match);

static struct i2c_driver hifibunny_codec_i2c_driver = {
	.driver = {
		.name           = "hifibunny-codec-i2c",
		.owner          = THIS_MODULE,
		.of_match_table = of_match_ptr(hifibunny_codec_of_match),
	},
	.probe    = hifibunny_codec_i2c_probe,
	.remove   = hifibunny_codec_i2c_remove,
	.id_table = hifibunny_codec_i2c_id,
};
module_i2c_driver(hifibunny_codec_i2c_driver);


MODULE_DESCRIPTION("ASoC Hifibunny Q2M codec driver");
MODULE_AUTHOR("Jiang Li");
MODULE_LICENSE("GPL");
