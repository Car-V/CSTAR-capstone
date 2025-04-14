#include <linux/module.h>
#define INCLUDE_VERMAGIC
#include <linux/build-salt.h>
#include <linux/elfnote-lto.h>
#include <linux/vermagic.h>
#include <linux/compiler.h>

BUILD_SALT;
BUILD_LTO_INFO;

MODULE_INFO(vermagic, VERMAGIC_STRING);
MODULE_INFO(name, KBUILD_MODNAME);

__visible struct module __this_module
__section(".gnu.linkonce.this_module") = {
	.name = KBUILD_MODNAME,
	.init = init_module,
#ifdef CONFIG_MODULE_UNLOAD
	.exit = cleanup_module,
#endif
	.arch = MODULE_ARCH_INIT,
};

#ifdef CONFIG_RETPOLINE
MODULE_INFO(retpoline, "Y");
#endif

static const struct modversion_info ____versions[]
__used __section("__versions") = {
	{ 0xad0962d1, "module_layout" },
	{ 0xdc5391c1, "platform_driver_unregister" },
	{ 0x8a81cb50, "__platform_driver_register" },
	{ 0xeb7416d5, "devm_snd_soc_register_component" },
	{ 0x9fb3ba2, "devm_gpio_request_one" },
	{ 0x4687a005, "of_get_named_gpio_flags" },
	{ 0xd9601d09, "of_match_device" },
	{ 0x67e44ba6, "devm_regulator_bulk_get" },
	{ 0x1ad97d9c, "__dynamic_dev_dbg" },
	{ 0x59ba780, "devm_kmalloc" },
	{ 0x77e7281f, "regulator_bulk_disable" },
	{ 0xf93d5faf, "_dev_info" },
	{ 0xabf9e9d3, "regulator_bulk_enable" },
	{ 0xf9a482f9, "msleep" },
	{ 0xbcf400f6, "gpiod_set_raw_value" },
	{ 0x460017f9, "gpio_to_desc" },
	{ 0xf41141c, "_dev_err" },
	{ 0xfe791560, "snd_soc_unregister_component" },
};

MODULE_INFO(depends, "snd-soc-core");

MODULE_ALIAS("of:N*T*Cakm,ak5720");
MODULE_ALIAS("of:N*T*Cakm,ak5720C*");

MODULE_INFO(srcversion, "166BE1B5677FE44C26B79D2");
