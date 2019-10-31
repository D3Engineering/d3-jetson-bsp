/*
 * Copyright (c) 2015-2016, NVIDIA CORPORATION.  All rights reserved.
 * Author: Laxman Dewangan <ldewangan@nvidia.com>
 *
 * Copyright (c) 2019, D3 Engineering  All rights reserved.
 * Author: Tyler Hart <thart@d3engineering.com>
 * Author: Christopher White <cwhite@d3engineering.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
 * more details.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/proc_fs.h>
#include <linux/of.h>
#include <linux/of_fdt.h>
#include <d3/of_private.h>

#ifdef CONFIG_D3_CONFIG_EEPROM
#include <d3/config-eeprom.h>
#endif

#define MODULE_NAME "overlay-loader"

#define MAXIMUM_FNAME_LENGTH		300
#define KERN_PARAM_NAME			"active_overlays="
#define DEVICETREE_PROPERTY_NAME	"active_overlays"

/**
 * struct kparam_element - node of a linked list holding strings
 * @value: A pointer to a string, allocated using kmalloc or kzalloc
 * @next: The next-element pointer
 */

struct kparam_element
{
	char* value;
	struct kparam_element *next;
};

/**
 * free_kparam_element - free a struct kparam_element and its contained string
 * @kpe: The element to free, or NULL
 */

void free_kparam_element(struct kparam_element *kpe)
{
	if (!kpe)
		return;

	kfree(kpe->value);
	kfree(kpe);
}

/**
 * free_kparam_list - free a full linked list of struct kparam_element
 * @kpe: The head of the list to free, or NULL
 *
 * This free @kpe and all following elements.
 */

void free_kparam_list(struct kparam_element *kpe)
{
	struct kparam_element *kpe_next;
	while (kpe != NULL) {
		kpe_next = kpe->next;
		free_kparam_element(kpe);
		kpe = kpe_next;
	}
}

static struct property *__of_copy_property(const struct property *prop,
					   void *new_value, int val_len,
					   gfp_t flags)
{
	struct property *propn;
	int nlen;
	void *nval;

	propn = kzalloc(sizeof(*propn), flags);
	if (!propn)
		return NULL;

	propn->name = kstrdup(prop->name, flags);
	if (!propn->name)
		goto err_fail_name;

	nlen = (new_value) ? val_len : prop->length;
	nval = (new_value) ? new_value : prop->value;
	if (nlen > 0) {
		propn->value = kzalloc(nlen, flags);
		if (!propn->value)
			goto err_fail_value;
		memcpy(propn->value, nval, nlen);
		propn->length = nlen;
	}
	return propn;

err_fail_value:
	kfree(propn->name);
err_fail_name:
	kfree(propn);
	return NULL;
}


static void free_property(struct property *pp)
{
	if (!pp)
		return;

	kfree(pp->name);
	kfree(pp->value);
	kfree(pp);
}

static struct device_node *of_get_child_by_last_name(struct device_node *node,
						     const char *name)
{
	struct device_node *child;

	for_each_child_of_node(node, child) {
		const char *lname = strrchr(child->full_name, '/');

		if (!strcmp(lname + 1, name))
			return child;
	}

	return NULL;
}


struct device_node *create_simple_device_node_ol(const char *path,
					      const char *add_name,
					      size_t data_size)
{
	struct device_node *new_np;

	new_np = kzalloc(sizeof(*new_np), GFP_KERNEL);
	if (!new_np)
		return NULL;

	new_np->full_name = kasprintf(GFP_KERNEL, "%s/%s", path, add_name);
	new_np->name = kasprintf(GFP_KERNEL, "%s", add_name);

	if (data_size) {
		new_np->data = kzalloc(data_size, GFP_KERNEL);
		if (!new_np->data)
			goto clean;
	}

	return new_np;

clean:
	kfree(new_np->full_name);
	kfree(new_np->name);
	kfree(new_np);
	return NULL;
}


struct device_node *duplicate_single_node_ol(struct device_node *np,
					  const char *base_dir,
					  const char *path,
					  const char *new_name)
{
	struct device_node *dup;
	struct property *pp, *new_pp;
	int ret;
	const char *add_name;
	char fname[MAXIMUM_FNAME_LENGTH + 1] = {};

	dup = kzalloc(sizeof(*dup), GFP_KERNEL);
	if (!dup)
		return NULL;

	if (new_name) {
		add_name = new_name;
	} else {
		add_name = strrchr(np->full_name, '/');
		add_name++;
	}

	if (path) {
		strncpy(fname, path, MAXIMUM_FNAME_LENGTH);
	} else {
		const char *lname = strrchr(np->full_name, '/');
		int llen = strlen(np->full_name) - strlen(lname);

		strncpy(fname, np->full_name, MAXIMUM_FNAME_LENGTH);
		fname[llen] = '\0';
	}

	if (base_dir)
		dup->full_name = kasprintf(GFP_KERNEL, "%s%s/%s",
					   base_dir, fname, add_name);
	else
		dup->full_name = kasprintf(GFP_KERNEL, "%s/%s",
					   fname, add_name);

	of_node_init(dup);

	for_each_property_of_node(np, pp) {
		if (!strcmp(pp->name, "name"))
			new_pp = __of_copy_property(pp, (void *)add_name,
						    strlen(add_name),
						    GFP_KERNEL);
		else
			new_pp = __of_copy_property(pp, NULL, 0, GFP_KERNEL);
		if (!new_pp) {
			kfree(dup->full_name);
			kfree(dup);
			return NULL;
		}

		ret = of_add_property(dup, new_pp);
		if (ret < 0) {
			pr_err("Prop %s can not be added on node %s\n",
			       new_pp->name, dup->full_name);
			free_property(new_pp);
			kfree(dup->full_name);
			kfree(dup);
			return NULL;
		}
	}

	dup->name = __of_get_property(dup, "name", NULL) ? : "<NULL>";
	dup->type = __of_get_property(dup, "device_type", NULL) ? : "<NULL>";

	return dup;
}

struct device_node *get_copy_of_node_ol(struct device_node *np,
				     const char *base_dir,
				     const char *path, const char *new_name)
{
	struct device_node *dup;
	struct device_node *child, *child_dup;
	struct device_node *prev_child = NULL;

	dup = duplicate_single_node_ol(np, base_dir, path, new_name);
	if (!dup)
		return NULL;

	for_each_child_of_node(np, child) {
		child_dup = get_copy_of_node_ol(child, NULL, dup->full_name, NULL);
		if (!child_dup) {
			kfree(dup);
			return NULL;
		}
		child_dup->parent = dup;
		child_dup->sibling = NULL;
		if (!prev_child)
			dup->child = child_dup;
		else
			prev_child->sibling = child_dup;
		prev_child = child_dup;
	}

	return dup;
}


static struct property *__of_string_append(struct device_node *target,
					   struct property *prop)
{
	struct property *new_prop, *tprop;
	const char *tprop_name, *curr_str;
	int slen, tlen, lenp;

	tprop_name = of_prop_next_string(prop, NULL);
	if (!tprop_name)
		return NULL;

	new_prop = kzalloc(sizeof(*new_prop), GFP_KERNEL);
	if (!new_prop)
		return NULL;

	new_prop->name = kstrdup(tprop_name, GFP_KERNEL);
	if (!new_prop->name)
		goto err_fail_name;

	curr_str = of_prop_next_string(prop, tprop_name);
	for (slen = 0; curr_str; curr_str = of_prop_next_string(prop, curr_str))
		slen += strlen(curr_str);

	tprop = of_find_property(target, tprop_name, &lenp);
	tlen = (tprop) ? tprop->length : 0;

	new_prop->value = kmalloc(slen + tlen, GFP_KERNEL);
	if (!new_prop->value)
		goto err_fail_value;

	if (tlen)
		memcpy(new_prop->value, tprop->value, tlen);

	if (slen) {
		curr_str = of_prop_next_string(prop, tprop_name);
		memcpy(new_prop->value + tlen, curr_str, slen);
	}

	new_prop->length = slen + tlen;

	return new_prop;

err_fail_value:
	kfree(new_prop->name);
err_fail_name:
	kfree(new_prop);

	return NULL;
}

static int do_property_override_from_overlay(struct device_node *target,
					     struct device_node *overlay)
{
	struct property *prop;
	struct property *tprop;
	struct property *new_prop;
	const char *pval;
	int lenp = 0;
	int ret;

	pr_debug("Update properties from %s to %s\n", overlay->full_name,
		 target->full_name);

	for_each_property_of_node(overlay, prop) {
		/* Skip those we do not want to proceed */
		if (!strcmp(prop->name, "name") ||
			!strcmp(prop->name, "phandle") ||
			!strcmp(prop->name, "linux,phandle"))
				continue;
		if (!strcmp(prop->name, "delete-target-property")) {
			if (prop->length <= 0)
				continue;
			pval = (const char *)prop->value;
			pr_info("Removing Prop %s from target %s\n",
				pval, target->full_name);
			tprop = of_find_property(target, pval, &lenp);
			if (tprop)
				of_remove_property(target, tprop);
			continue;
		}

		if (!strcmp(prop->name, "append-string-property")) {
			if (prop->length <= 0)
				continue;

			new_prop = __of_string_append(target, prop);
			if (!new_prop) {
				pr_err("Prop %s can not be appended\n",
					of_prop_next_string(prop, NULL));
				return -EINVAL;
			}
			goto add_prop;
		}

		new_prop = __of_copy_property(prop, NULL, 0, GFP_KERNEL);
		if (!new_prop) {
			pr_err("Prop %s can not be duplicated\n",
				prop->name);
			return -EINVAL;
		}

add_prop:
		tprop = of_find_property(target, new_prop->name, &lenp);
		if (!tprop) {
			ret = of_add_property(target, new_prop);
			if (ret < 0) {
				pr_err("Prop %s can not be added on node %s\n",
					new_prop->name, target->full_name);
				goto cleanup;
			}
		} else {
			ret = of_update_property(target, new_prop);
			if (ret < 0) {
				pr_err("Prop %s can not be updated on node %s\n",
					new_prop->name, target->full_name);
				goto cleanup;
			}
		}
	}

	return 0;

cleanup:
	free_property(new_prop);
	return ret;
}


static int do_property_overrides(struct device_node *target,
				 struct device_node *overlay)
{
	struct device_node *tchild, *ochild;
	const char *address_name;
	int ret;

	ret = do_property_override_from_overlay(target, overlay);
	if (ret < 0) {
		pr_err("Target %s update with overlay %s failed: %d\n",
			target->name, overlay->name, ret);
		return ret;
	}

	for_each_child_of_node(overlay, ochild) {
		address_name = strrchr(ochild->full_name, '/');
		tchild = of_get_child_by_last_name(target, address_name + 1);
		if (!tchild) {
			pr_err("Overlay node %s not found in target node %s\n",
				ochild->full_name, target->full_name);
			continue;
		}
		ret = do_property_overrides(tchild, ochild);
		if (ret < 0) {
			pr_err("Target %s update with overlay %s failed: %d\n",
				tchild->name, ochild->name, ret);
			return ret;
		}
	}
	return 0;
}

static int handle_properties_overrides(struct device_node *np,
				       struct device_node *target)
{
	struct device_node *overlay;
	int ret;

	if (!target) {
		target = of_parse_phandle(np, "target", 0);
		if (!target) {
			pr_err("Node %s does not have target node\n",
				np->name);
			return -EINVAL;
		}
	}

	overlay = of_get_child_by_name(np, "_overlay_");
	if (!overlay) {
		pr_err("Node %s does not have Overlay\n", np->name);
		return -EINVAL;
	}

	ret = do_property_overrides(target, overlay);
	if (ret < 0) {
		pr_err("Target %s update with overlay %s failed: %d\n",
			target->name, overlay->name, ret);
		return -EINVAL;
	}

	return 0;
}

/**
 * parse_parameters() - parse a string of overlay IDs into a linked list
 * @input_k_elem: where the return value should be stored
 * @kparam_start: the text to parse, terminated by \0 or a space (\x20).
 * 			This text does not include any `active_overlays=`
 * 			or other introductory text.
 *
 * Parses a comma-separated list of strings, e.g., "foo_0,bar_1".  Each
 * comma-separated item is placed in its own linked-list node.
 *
 * Return: 0 on success; a negated error code on failure.
 */

static int parse_parameters(struct kparam_element **input_k_elem,
		const char* kparam_start)
{

	struct kparam_element *cur_k_elem;
	struct kparam_element *new_k_elem;
	const char *kparam_end;
	int kparam_len;
	bool elem_end;
	bool params_end;

	// find param values
	kparam_end = kparam_start;
	cur_k_elem = NULL;
	pr_debug(MODULE_NAME ": Parsing params: >>%s<<\n", kparam_start);

	while (1) {
		params_end = (*kparam_end == '\0' || *kparam_end == ' ');
		elem_end = (params_end || *kparam_end == ',');
		if (elem_end) {
			kparam_len = strlen(kparam_start) - strlen(kparam_end);
			// manage linked list
			new_k_elem = kzalloc(sizeof(*(cur_k_elem->next)),
					GFP_KERNEL);
			new_k_elem->next = cur_k_elem;
			cur_k_elem = new_k_elem;
			cur_k_elem->value = kzalloc(sizeof(char) *
					(kparam_len + 1), GFP_KERNEL);
			strncpy(cur_k_elem->value, kparam_start, kparam_len);
			pr_debug(MODULE_NAME ":    found element %s\n", cur_k_elem->value);
			// move start pointer over separator char
			kparam_start = kparam_end;
			kparam_start++;
		}
		if (params_end)
			break;
		kparam_end++;
	}
	pr_debug(MODULE_NAME ": Done parsing params\n");

	// verify param values exist
	if (!cur_k_elem)
		return -EINVAL;
	*input_k_elem = cur_k_elem;
	return 0;
}

/**
 * get_kernel_parameters() - extract active_overlays values from the kernel command line.
 * @input_k_elem: where the return value should be stored.
 * @param_name: the name of the command-line parameter to check
 *
 * Parses the kernel command line to locate the names of any overlays
 * that should be activated.  Fills in @input_k_elem with a pointer to the
 * head of a linked list of results if successful.
 *
 * Return: 0 on success; a negated error code on failure.
 */

static int get_kernel_parameters(struct kparam_element **input_k_elem,
		const char* param_name)
{
	char *kparam_start;

	kparam_start = saved_command_line;
	*input_k_elem = NULL;

	// find start of parameter
	while (strlen(param_name) <= strlen(kparam_start)
			&& strncmp(param_name, kparam_start,
			strlen(param_name))) {
		kparam_start++;
	}

	// verify param name exists
	if (strncmp(param_name, kparam_start, strlen(param_name)) != 0)
		return -EINVAL;

	// Increment past param name
	kparam_start += strlen(param_name);

	return parse_parameters(input_k_elem, kparam_start);
}

/**
 * get_dt_parameters() - extract active_overlays values from the device tree
 * @input_k_elem: where the return value should be stored.
 * @np: the device-tree node including an active_overlays property
 *  	(e.g., /overlay-loader)
 *
 * Parses the kernel command line to locate the names of any overlays
 * that should be activated.  Fills in @input_k_elem with a pointer to the
 * head of a linked list of results if successful.
 *
 * Return: 0 on success; a negated error code on failure.
 */

static int get_dt_parameters(struct kparam_element **input_k_elem,
		struct device_node *np)
{
	const char *prop;
	prop = of_get_property(np, DEVICETREE_PROPERTY_NAME, NULL);
	if (!prop)
		return -EINVAL;
	return parse_parameters(input_k_elem, prop);
}

#ifdef CONFIG_D3_CONFIG_EEPROM

/**
 * get_config_eeprom_parameters() - extract active_overlays values from the
 * 					config-eeprom
 * @input_k_elem: where the return value should be stored.
 *
 * If there is any configuration-eeprom data loaded, parses it to locate the
 * names of any overlays that should be activated.  Fills in @input_k_elem with
 * a pointer to the head of a linked list of results if successful.
 *
 * Return:
 *
 * - On success with data, returns 0 and fills in *@input_k_elem.
 *
 * - On success with no data, returns 0 and sets *@input_k_elem = NULL.
 *
 * - On error, returns a negated error code and sets *@input_k_elem = NULL.
 */

static int get_config_eeprom_parameters(struct kparam_element **input_k_elem)
{
	struct config_eeprom_data *data;
	static bool already_reported = false;
	*input_k_elem = NULL;

	data = config_eeprom_get_data();
	if(!data) {
		if(!already_reported)
			pr_info(MODULE_NAME ": No config-eeprom data");
		already_reported = true;
		return -ENODEV;
	}

	// Basic overrun guard - TODO replace this with more robust code
	data->overlay_loader_params[CONFIG_EEPROM_DATA_SIZE-1] = '\0';

	return parse_parameters(input_k_elem, data->overlay_loader_params);
}

#endif // CONFIG_D3_CONFIG_EEPROM

/**
 * check_for_active_overlay - check whether an overlay name is active
 * @kparams_ptr: pointer to linked list of which overlays are active
 * @np: the device-tree node of this fragment (/overlay-loader/<fragment>)
 * @source: text naming the source of @kparams, for logging.
 *
 * This traverses @kparams and, for each kparam, checks whether @np
 * is active for the respective value.
 *
 * If @kparams_ptr, @np, and @source are all non-NULL, all nodes in
 * @kparams are freed by this function, and *@kparams_ptr is set to NULL.
 *
 * Return: false on parameter error; whether or not the overlay is active,
 * otherwise.
 */

static bool check_for_active_overlay(struct kparam_element **kparams_ptr,
		struct device_node *np, const char *source)
{
	struct kparam_element *kparams;
	struct property *prop;
	const char *bname;
	struct kparam_element *last_param = NULL;
	bool found = false;

	if(!kparams_ptr || !np || !source)
		return false;

	kparams = *kparams_ptr;

	if(np->full_name) {
		pr_info(MODULE_NAME ": checking whether %s enables %s\n",
				source, np->full_name);
	} else {
		pr_info(MODULE_NAME ": checking for %s parameters\n", source);
	}

	while (kparams) {
		of_property_for_each_string(np, "param", prop, bname) {
			found = strcmp(kparams->value, bname) == 0;
			if (found) {
				pr_info("node %s match with %s parameter %s\n",
					np->full_name, source, kparams->value);
				free_kparam_list(kparams);
				goto search_done;	// double break
			}
		}
		last_param = kparams;
		kparams = kparams->next;
		free_kparam_element(last_param);
	}

search_done:
	*kparams_ptr = NULL;
	return found;
}

/**
 * process_fragment() - modify device tree if the fragment is active
 * @np: the device-tree node of this fragment (/overlay-loader/<fragment>)
 * @overlay_loader_node: the device-tree node for /overlay-loader
 *
 * Process a fragment.  If the value in /overlay-loader/<fragment>/param is an
 * active overlay, apply that fragment's override@n nodes to the device tree.
 * The overlay is active if it is listed in any of:
 *
 * - The kernel command line's active_overlays parameter
 *
 * - The active_overlays parameter in the device tree in
 *   /overlay-loader/active_overlays
 *
 * - The configuration EEPROM loaded by the config-eeprom driver
 *   (if the config-eeprom driver is loaded)
 *
 * Return: 0 on success, or a negated error on failure.
 * Note that "success" can mean that an active fragment was applied or that
 * an inactive fragment was skipped.
 */

static int __init process_fragment(struct device_node *np,
		struct device_node *overlay_loader_node)
{
	struct device_node *cnp;
	struct kparam_element *kparams = NULL;
	//char *target = NULL;
	int target_param_count;
	int nchild;
	bool found = false;

	target_param_count = of_property_count_strings(np, "param");
	if (target_param_count <=0) {
		pr_err("Node %s does not have any params\n",
			np->name);
		return -EINVAL;
	}

	nchild = of_get_child_count(np);
	if (!nchild) {
		pr_err("Node %s does not have Overlay child\n", np->name);
		return -EINVAL;
	}

	// Check kernel command line
	if(!found && (get_kernel_parameters(&kparams, KERN_PARAM_NAME) == 0))
		found = check_for_active_overlay(&kparams, np, "kernel");

	// Check device tree
	if(!found && (get_dt_parameters(&kparams, overlay_loader_node) == 0))
		found = check_for_active_overlay(&kparams, np, "device-tree");

#ifdef CONFIG_D3_CONFIG_EEPROM

	// Check configuration EEPROM
	if(!found && (get_config_eeprom_parameters(&kparams) == 0))
		found = check_for_active_overlay(&kparams, np, "config-eeprom");

#endif // CONFIG_D3_CONFIG_EEPROM

	if (!found)
		return 0;

	for_each_child_of_node(np, cnp) {
		handle_properties_overrides(cnp, NULL);
	}

	return 0;
}


static int __init overlay_loader_init(void)
{
	struct device_node *overlay_loader_node;
	struct device_node *child;
	int ret;

	pr_info("Initializing overlay-loader\n");

	overlay_loader_node = of_find_node_by_path("/overlay-loader");
	if (!overlay_loader_node) {
		pr_info("Overlay-loader not available, no root node\n");
		return 0;
	}

	if (!of_device_is_available(overlay_loader_node)) {
		pr_info("Overlay-loader status disabled\n");
		return 0;
	}

	for_each_available_child_of_node(overlay_loader_node, child) {
		ret = process_fragment(child, overlay_loader_node);
		if (ret < 0)
			pr_err("Error in parsing node %s: %d\n",
				child->full_name, ret);
	}
	return 0;
}

/* Initialize the overlay loader at level 5 (fs).  This is after subsys (#4),
 * so nvmem is available, and is before device (#6), when devices
 * are initialized based on the DTB state. */

fs_initcall(overlay_loader_init);

MODULE_DESCRIPTION("A driver that will apply device tree fragments based on values from the kernel command line, the device tree, or a configuration EEPROM.");
MODULE_AUTHOR("Laxman Dewangan <ldewangan@nvidia.com>");
MODULE_AUTHOR("Tyler Hart <thart@d3engineering.com>");
MODULE_AUTHOR("Christopher White <cwhite@d3engineering.com>");
MODULE_LICENSE("GPL v2");
