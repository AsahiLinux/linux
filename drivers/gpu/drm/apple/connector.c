// SPDX-License-Identifier: GPL-2.0+ OR MIT
/*
 * Copyright (C) The Asahi Linux Contributors
 */

#include <linux/debugfs.h>
#include <linux/module.h>
#include <linux/seq_file.h>
#include <linux/string_helpers.h>
#include <linux/uaccess.h>

#include <drm/drm_managed.h>

#include "connector.h"
#include "dcp-internal.h"

enum dcp_chunk_type {
	DCP_CHUNK_COLOR_ELEMENTS,
	DCP_CHUNK_TIMING_ELELMENTS,
	DCP_CHUNK_DISPLAY_ATTRIBUTES,
	DCP_CHUNK_TRANSPORT,
	DCP_CHUNK_NUM_TYPES,
};

static int chunk_show(struct seq_file *m,
		      enum dcp_chunk_type chunk_type)
{
	struct apple_connector *apple_con = m->private;
	struct dcp_chunks *chunk = NULL;

	mutex_lock(&apple_con->chunk_lock);

	switch (chunk_type) {
	case DCP_CHUNK_COLOR_ELEMENTS:
		chunk = &apple_con->color_elements;
		break;
	case DCP_CHUNK_TIMING_ELELMENTS:
		chunk = &apple_con->timing_elements;
		break;
	case DCP_CHUNK_DISPLAY_ATTRIBUTES:
		chunk = &apple_con->display_attributes;
		break;
	case DCP_CHUNK_TRANSPORT:
		chunk = &apple_con->transport;
		break;
	default:
		break;
	}

	if (chunk)
                seq_write(m, chunk->data, chunk->length);

	mutex_unlock(&apple_con->chunk_lock);

	return 0;
}

#define CONNECTOR_DEBUGFS_ENTRY(name, type) \
static int chunk_ ## name ## _show(struct seq_file *m, void *data) \
{ \
        return chunk_show(m, type); \
} \
static int chunk_ ## name ## _open(struct inode *inode, struct file *file) \
{ \
        return single_open(file,  chunk_ ## name ## _show, inode->i_private); \
} \
static const struct file_operations chunk_ ## name ## _fops = { \
        .owner = THIS_MODULE, \
        .open = chunk_ ## name ## _open, \
        .read = seq_read, \
        .llseek = seq_lseek, \
        .release = single_release, \
}

CONNECTOR_DEBUGFS_ENTRY(color, DCP_CHUNK_COLOR_ELEMENTS);
CONNECTOR_DEBUGFS_ENTRY(timing, DCP_CHUNK_TIMING_ELELMENTS);
CONNECTOR_DEBUGFS_ENTRY(display_attribs, DCP_CHUNK_DISPLAY_ATTRIBUTES);
CONNECTOR_DEBUGFS_ENTRY(transport, DCP_CHUNK_TRANSPORT);

void apple_connector_debugfs_init(struct drm_connector *connector, struct dentry *root)
{
	struct apple_connector *apple_con = to_apple_connector(connector);

        debugfs_create_file("ColorElements", 0444, root, apple_con,
                            &chunk_color_fops);
        debugfs_create_file("TimingElements", 0444, root, apple_con,
                            &chunk_timing_fops);
        debugfs_create_file("DisplayAttributes", 0444, root, apple_con,
                            &chunk_display_attribs_fops);
        debugfs_create_file("Transport", 0444, root, apple_con,
                            &chunk_transport_fops);
}
EXPORT_SYMBOL(apple_connector_debugfs_init);

static void dcp_connector_set_dict(struct apple_connector *connector,
				   struct dcp_chunks *dict,
				   struct dcp_chunks *chunks)
{
	if (dict->data)
		devm_kfree(&connector->dcp->dev, dict->data);

	*dict = *chunks;
}

void dcp_connector_update_dict(struct apple_connector *connector, const char *key,
			       struct dcp_chunks *chunks)
{
	mutex_lock(&connector->chunk_lock);
	if (!strcmp(key, "ColorElements"))
		dcp_connector_set_dict(connector, &connector->color_elements, chunks);
	else if (!strcmp(key, "TimingElements"))
		dcp_connector_set_dict(connector, &connector->timing_elements, chunks);
	else if (!strcmp(key, "DisplayAttributes"))
		dcp_connector_set_dict(connector, &connector->display_attributes, chunks);
	else if (!strcmp(key, "Transport"))
		dcp_connector_set_dict(connector, &connector->transport, chunks);

	chunks->data = NULL;
	chunks->length = 0;

	mutex_unlock(&connector->chunk_lock);
}
