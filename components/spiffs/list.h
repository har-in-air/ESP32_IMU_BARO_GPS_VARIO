/*
 * Lua RTOS, list data structure
 *
 * Copyright (C) 2015 - 2017
 * IBEROXARXA SERVICIOS INTEGRALES, S.L. & CSS IBÉRICA, S.L.
 * 
 * Author: Jaume Olivé (jolive@iberoxarxa.com / jolive@whitecatboard.org)
 * 
 * All rights reserved.  
 *
 * Permission to use, copy, modify, and distribute this software
 * and its documentation for any purpose and without fee is hereby
 * granted, provided that the above copyright notice appear in all
 * copies and that both that the copyright notice and this
 * permission notice and warranty disclaimer appear in supporting
 * documentation, and that the name of the author not be used in
 * advertising or publicity pertaining to distribution of the
 * software without specific, written prior permission.
 *
 * The author disclaim all warranties with regard to this
 * software, including all implied warranties of merchantability
 * and fitness.  In no event shall the author be liable for any
 * special, indirect or consequential damages or any damages
 * whatsoever resulting from loss of use, data or profits, whether
 * in an action of contract, negligence or other tortious action,
 * arising out of or in connection with the use or performance of
 * this software.
 */

#ifndef _LIST_H
#define	_LIST_H

#include <stdint.h>
#include "mutex.h"

#ifdef __cplusplus
extern "C" {
#endif

struct splist {
    struct mtx mutex;
    struct splist_index *index;
    struct splist_index *free;
    uint8_t indexes;
    uint8_t first_index;
};

struct splist_index {
    void *item;
    uint8_t index;
    uint8_t deleted;
    struct splist_index *next;
};

void splist_init(struct splist *list, int first_index);
int splist_add(struct splist *list, void *item, int *item_index);
int splist_get(struct splist *list, int index, void **item);
int splist_remove(struct splist *list, int index, int destroy);
int splist_first(struct splist *list);
int splist_next(struct splist *list, int index);
void splist_destroy(struct splist *list, int items);

#ifdef __cplusplus
}
#endif

#endif	/* LIST_H */

