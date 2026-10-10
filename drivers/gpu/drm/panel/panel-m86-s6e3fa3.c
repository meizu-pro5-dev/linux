// SPDX-License-Identifier: GPL-2.0-only
/* Meizu PRO 5 S6E3FA3. Sequence from meizuosc/m576 ubuntu 5a0c442. */
#include <linux/backlight.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>

struct m86_panel {
 struct drm_panel panel;
 struct mipi_dsi_device *dsi;
 struct gpio_desc *reset;
 struct regulator_bulk_data supplies[2];
 struct backlight_device *backlight;
 bool prepared;
};
static inline struct m86_panel *to_m86(struct drm_panel *p)
{ return container_of(p, struct m86_panel, panel); }
static int m86_brightness(struct backlight_device *bl)
{
 struct m86_panel *p = bl_get_data(bl);
 u8 data[] = { 0x51, backlight_get_brightness(bl) };
 if (!p->prepared) return 0;
 return mipi_dsi_dcs_write_buffer(p->dsi, data, sizeof(data)) < 0 ? -EIO : 0;
}
static const struct backlight_ops m86_bl_ops = {
 .options = BL_CORE_SUSPENDRESUME, .update_status = m86_brightness,
};
struct m86_cmd { u8 data[3]; u8 length, delay; bool generic; };
static const struct m86_cmd init_cmds[] = {
 {{0xf0,0x5a,0x5a},3,0,true}, {{0x11},1,20},
 {{0xf9,0x12},2,0}, {{0xf0,0xa5,0xa5},3,0,true},
 {{0x35,0x00},2,20}, {{0xfc,0x5a,0x5a},3,0,true},
 {{0xb0,0x1e},2,0}, {{0xfd,0x94},2,0},
 {{0xfc,0xa5,0xa5},3,0,true}, {{0x53,0x20},2,0},
 {{0x51,0x00},2,80}, {{0x55,0x00},2,10},
};
static int m86_prepare(struct drm_panel *panel)
{
 struct m86_panel *p = to_m86(panel);
 int ret, i;
 u8 id[3];
 if (p->prepared) return 0;
 ret = regulator_bulk_enable(2, p->supplies);
 if (ret) return ret;
 gpiod_set_value_cansleep(p->reset, 0);
 usleep_range(5000,6000);
 gpiod_set_value_cansleep(p->reset, 1);
 usleep_range(5000,6000);
 gpiod_set_value_cansleep(p->reset, 0);
 usleep_range(10000,11000);
 for (i=0; i<ARRAY_SIZE(init_cmds); i++) {
  const struct m86_cmd *c=&init_cmds[i];
  ret = c->generic ? mipi_dsi_generic_write(p->dsi,c->data,c->length) :
        mipi_dsi_dcs_write_buffer(p->dsi,c->data,c->length);
  if (ret < 0) goto fail;
  if (c->delay) msleep(c->delay);
 }
 ret=mipi_dsi_set_maximum_return_packet_size(p->dsi,sizeof(id));
 if (ret>=0) ret=mipi_dsi_dcs_read(p->dsi,0x04,id,sizeof(id));
 if (ret==sizeof(id)) dev_info(panel->dev,"S6E3FA3 ID %02x %02x %02x\n",id[0],id[1],id[2]);
 else dev_warn(panel->dev,"panel ID read returned %d\n",ret);
 p->prepared=true;
 return 0;
fail:
 gpiod_set_value_cansleep(p->reset,1);
 regulator_bulk_disable(2,p->supplies);
 return ret;
}
static int m86_enable(struct drm_panel *panel)
{ return mipi_dsi_dcs_set_display_on(to_m86(panel)->dsi); }
static int m86_disable(struct drm_panel *panel)
{
 int ret=mipi_dsi_dcs_set_display_off(to_m86(panel)->dsi);
 msleep(10); return ret;
}
static int m86_unprepare(struct drm_panel *panel)
{
 struct m86_panel *p=to_m86(panel);
 if (!p->prepared) return 0;
 mipi_dsi_dcs_enter_sleep_mode(p->dsi); msleep(150);
 gpiod_set_value_cansleep(p->reset,1);
 p->prepared=false;
 return regulator_bulk_disable(2,p->supplies);
}
static int m86_modes(struct drm_panel *panel,struct drm_connector *connector)
{
 static const struct drm_display_mode mode = {
  .clock=125216, .hdisplay=1080, .hsync_start=1081, .hsync_end=1082, .htotal=1083,
  .vdisplay=1920, .vsync_start=1925, .vsync_end=1926, .vtotal=1927,
  .width_mm=72,.height_mm=147,
 };
 struct drm_display_mode *m=drm_mode_duplicate(connector->dev,&mode);
 if (!m) return -ENOMEM;
 m->type=DRM_MODE_TYPE_DRIVER|DRM_MODE_TYPE_PREFERRED;
 drm_mode_set_name(m); drm_mode_probed_add(connector,m);
 connector->display_info.width_mm=72; connector->display_info.height_mm=147;
 return 1;
}
static const struct drm_panel_funcs m86_funcs = {
 .prepare=m86_prepare,.enable=m86_enable,.disable=m86_disable,
 .unprepare=m86_unprepare,.get_modes=m86_modes,
};
static int m86_probe(struct mipi_dsi_device *dsi)
{
 struct m86_panel *p;
 struct backlight_properties props={.type=BACKLIGHT_RAW,.max_brightness=255,.brightness=68};
 int ret;
 p=devm_kzalloc(&dsi->dev,sizeof(*p),GFP_KERNEL);
 if (!p) return -ENOMEM;
 p->dsi=dsi;
 p->supplies[0].supply="vci"; p->supplies[1].supply="vdd3";
 ret=devm_regulator_bulk_get(&dsi->dev,2,p->supplies);
 if (ret) return dev_err_probe(&dsi->dev,ret,"panel supplies\n");
 p->reset=devm_gpiod_get(&dsi->dev,"reset",GPIOD_OUT_HIGH);
 if (IS_ERR(p->reset)) return dev_err_probe(&dsi->dev,PTR_ERR(p->reset),"panel reset\n");
 p->backlight=devm_backlight_device_register(&dsi->dev,"m86-panel",&dsi->dev,p,&m86_bl_ops,&props);
 if (IS_ERR(p->backlight)) return PTR_ERR(p->backlight);
 drm_panel_init(&p->panel,&dsi->dev,&m86_funcs,DRM_MODE_CONNECTOR_DSI);
 p->panel.backlight=p->backlight;
 p->panel.prepare_prev_first=true;
 drm_panel_add(&p->panel);
 mipi_dsi_set_drvdata(dsi,p);
 dsi->lanes=2; dsi->format=MIPI_DSI_FMT_RGB888; dsi->mode_flags=0; /* vendor uses HS command transfers */
 ret=mipi_dsi_attach(dsi);
 if (ret) drm_panel_remove(&p->panel);
 return ret;
}
static void m86_remove(struct mipi_dsi_device *dsi)
{
 struct m86_panel *p=mipi_dsi_get_drvdata(dsi);
 mipi_dsi_detach(dsi); drm_panel_remove(&p->panel);
}
static const struct of_device_id m86_match[]={{.compatible="meizu,m86-s6e3fa3"},{}};
MODULE_DEVICE_TABLE(of,m86_match);
static struct mipi_dsi_driver m86_driver={
 .probe=m86_probe,.remove=m86_remove,
 .driver={.name="panel-m86-s6e3fa3",.of_match_table=m86_match},
};
module_mipi_dsi_driver(m86_driver);
MODULE_LICENSE("GPL");
