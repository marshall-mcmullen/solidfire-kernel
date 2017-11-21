/*
 * Copyright (C) 2011 Marvell Semiconductor, Inc
 *
 * Licensed under the MIT License
 * Alternatively, you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * See COPYING-GPL.txt for the GPL license text.
 *
 * MIT License:
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */
#ifndef _DLIST_H_
#define _DLIST_H_

/** \file dlist.h
 *
 * Header file to implement embeded double link list.
 */

/** \typedef dlist_t
 *
 * An embeded dlist structure.
 */
typedef struct dlist dlist_t;
struct dlist {
	dlist_t  *dlist_next;
	dlist_t  *dlist_prev;
};

/*
 * Macro to get the type of the emembeded link.
 */
#define DLIST_NEXT(ptr)                   ((ptr)->dlist_next)
#define DLIST_PREV(ptr)                   ((ptr)->dlist_prev)
#define DLIST_ENDP(ptr)                   ((ptr)->dlist_next == (ptr))

/*
 * Macros for dlist traversal.
 */
#define dlist_for_each(dlist, ptr)					\
	for (ptr = (dlist)->dlist_next; ptr != (dlist); ptr = ptr->dlist_next)

#define dlist_for_each_prev(dlist, ptr)					\
	for (ptr = (dlist)->dlist_prev; ptr != (dlist); ptr = ptr->dlist_prev)

#define dlist_for_each_safe(head, cur, nxt)                             \
	for (cur = (head)->dlist_next, nxt = cur->dlist_next;		\
			cur != (head);					\
		cur = nxt, nxt = (cur)->dlist_next)

/**
 * dlist_init
 * ----------
 * Initialize a link list dlist.
 *
 * \param dlist - the double linked list to initialize.
 */
static inline void
dlist_init(dlist_t *dlist)
{
	dlist->dlist_next = dlist->dlist_prev = dlist;
}

/**
 * dlist_empty
 * -----------
 * \return TRUE if a link list is empty, FALSE otherwise.
 */
static inline int
dlist_empty(dlist_t *dlist)
{
	return (dlist->dlist_next == dlist);
}

/**
 * dlist_add_between
 * -----------------
 * Add a dlist entry between two elements.
 */
static inline void
dlist_add_between(dlist_t *newl, dlist_t *prev, dlist_t *next)
{
	next->dlist_prev = newl;
	newl->dlist_next = next;
	newl->dlist_prev = prev;
	prev->dlist_next = newl;
}

/**
 * dlist_add_front
 * ---------------
 * Add a dlist entry in front of a list.
 *
 * \param dlist - the double list to insert.
 * \param entry - the new entry to add to the list.
 */
static inline void
dlist_add_front(dlist_t *dlist, dlist_t *entry)
{
	dlist_add_between(entry, dlist, dlist->dlist_next);
}

/**
 * dlist_add_back
 * --------------
 * Add a dlist entry to the back of a link list.
 *
 * \param dlist - the double list to insert.
 * \param entry - the new entry to add to the list.
 */
static inline void
dlist_add_back(dlist_t *dlist, dlist_t *entry)
{
	dlist_add_between(entry, dlist->dlist_prev, dlist);
}

/**
 * dlist_rm
 * --------
 * Remove a list entry out of its dlink list.
 *
 * \param entry - entry to remove out of the list.
 */
static inline void
dlist_rm(dlist_t *entry)
{
	entry->dlist_prev->dlist_next = entry->dlist_next;
	entry->dlist_next->dlist_prev = entry->dlist_prev;
}

/**
 * dlist_rm_safe
 * -------------
 * Remove the given entry and initialize its link pointers to NULL
 *
 * \param entry - the entry to remove out of the list.
 */
static inline void
dlist_rm_safe(dlist_t *entry)
{
	dlist_rm(entry);
	entry->dlist_prev = entry->dlist_next = NULL;
}

/**
 * dlist_rm_reinit
 * ---------------
 * Remove the given entry and reinitialize its link to itself
 *
 * \param entry - the entry to remove out of the list.
 */
static inline void
dlist_rm_reinit(dlist_t *entry)
{
	dlist_rm(entry);
	dlist_init(entry);
}


/**
 * dlist_replace
 * -------------
 * API to replace an entry (used to move a list to different head)
 *
 * \param old - old entry to be removed
 * \param new - new entry to be added
 */
static inline void
dlist_replace(dlist_t *old, dlist_t *new)
{
	dlist_init(new);

	if (dlist_empty(old))
		return;

	new->dlist_next = old->dlist_next;
	new->dlist_prev = old->dlist_prev;
	new->dlist_next->dlist_prev = new;
	new->dlist_prev->dlist_next = new;
}


/**
 * dlist_peek_front
 * ----------------
 * API to peek the first element of the list.
 *
 * \param dlist - the double linked list to peek from.
 * \return the first list element, NULL if empty list.
 */
static inline dlist_t *
dlist_peek_front(dlist_t *dlist)
{
	if (!dlist_empty(dlist)) {
		return (dlist->dlist_next);
	}
	return (NULL);
}

/**
 * dlist_peek_back
 * ---------------
 * API to peek the last element of the list.
 *
 * \param dlist - the double linked list to peek from.
 * \return the last list element, NULL if empty list.
 */
static inline dlist_t *
dlist_peek_back(dlist_t *dlist)
{
	if (!dlist_empty(dlist)) {
		return (dlist->dlist_prev);
	}
	return (NULL);
}

/**
 * dlist_rm_front
 * --------------
 * Remove the element from the front of a list; NULL if the list is empty.
 *
 * \param dlist - the double linked list to remove from.
 * \return the first list element, NULL if empty list.
 */
static inline dlist_t *
dlist_rm_front(dlist_t *dlist)
{
	dlist_t *res;

	if (!dlist_empty(dlist)) {
		res = dlist->dlist_next;
		dlist_rm_safe(res);
		return (res);
	}
	return (NULL);
}

/**
 * dlist_rm_back
 * -------------
 * Remove the element from the back of a list; NULL if the list is empty.
 *
 * \param dlist - the double linked list to remove from.
 * \return the first list element, NULL if empty list.
 */
static inline dlist_t *
dlist_rm_back(dlist_t *dlist)
{
	dlist_t *res;

	if (!dlist_empty(dlist)) {
		res = dlist->dlist_prev;
		dlist_rm_safe(res);
		return (res);
	}
	return (NULL);
}

#endif /* _DLIST_H_ */
