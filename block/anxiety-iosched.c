/*
 * Anxiety IO scheduler
 * Copyright (C) 2018 Draco (Tyler Nijmeh) <tylernij@gmail.com>
 */
#include <linux/blkdev.h>
#include <linux/elevator.h>
#include <linux/bio.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/init.h>

#define MAX_WRITES_STARVED 12

enum {ASYNC, SYNC};

struct anxiety_data {
	struct list_head queue[2][2];
	size_t writes_starved;
};

static void anxiety_merged_requests(struct request_queue *q, struct request *rq, struct request *next) {
	rq_fifo_clear(next);
}

static __always_inline struct request *anxiety_choose_request(struct anxiety_data *mdata) {
	// ensure that reads will always take priority unless writes are exceedingly starved
	bool starved = (mdata->writes_starved > MAX_WRITES_STARVED);

 	// sync read
	if (!starved && !list_empty(&mdata->queue[SYNC][READ])) {
		mdata->writes_starved++;
		return rq_entry_fifo(mdata->queue[SYNC][READ].next);
	}

	// sync write
	if (!list_empty(&mdata->queue[SYNC][WRITE])) {
		mdata->writes_starved = 0;
		return rq_entry_fifo(mdata->queue[SYNC][WRITE].next);
	}

	// async read
	if (!starved && !list_empty(&mdata->queue[ASYNC][READ])) {
		mdata->writes_starved++;
		return rq_entry_fifo(mdata->queue[ASYNC][READ].next);
	}

	// async write
	if (!list_empty(&mdata->queue[ASYNC][WRITE])) {
		mdata->writes_starved = 0;
		return rq_entry_fifo(mdata->queue[ASYNC][WRITE].next);
	}

	// all requests are finished
	mdata->writes_starved = 0;
	return NULL;
}

static int anxiety_dispatch(struct request_queue *q, int force) {
	struct request *rq = anxiety_choose_request(q->elevator->elevator_data);
	if (!rq)
		return 0;

	rq_fifo_clear(rq);
	elv_dispatch_add_tail(rq->q, rq);
	return 1;
}

static void anxiety_add_request(struct request_queue *q, struct request *rq) {
	const uint8_t sync = rq_is_sync(rq);
	const uint8_t read = rq_data_dir(rq);	
	list_add_tail(&rq->queuelist, &((struct anxiety_data *) q->elevator->elevator_data)->queue[sync][read]);
}

static struct request *anxiety_former_request(struct request_queue *q, struct request *rq) {
	const uint8_t sync = rq_is_sync(rq);
	const uint8_t read = rq_data_dir(rq);
	if (rq->queuelist.prev == &((struct anxiety_data *) q->elevator->elevator_data)->queue[sync][read])
		return NULL;
	return list_prev_entry(rq, queuelist);
}

static struct request *anxiety_latter_request(struct request_queue *q, struct request *rq) {
	const uint8_t sync = rq_is_sync(rq);
	const uint8_t read = rq_data_dir(rq);
	if (rq->queuelist.next == &((struct anxiety_data *) q->elevator->elevator_data)->queue[sync][read])
		return NULL;
	return list_next_entry(rq, queuelist);
}

static int anxiety_init_queue(struct request_queue *q, struct elevator_type *e) {
	struct anxiety_data *nd; 
	struct elevator_queue *eq = elevator_alloc(q, e);
	if (!eq)
		return -ENOMEM;

	nd = kmalloc_node(sizeof(*nd), GFP_KERNEL, q->node);
	if (!nd) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = nd;

	INIT_LIST_HEAD(&nd->queue[SYNC][READ]);
	INIT_LIST_HEAD(&nd->queue[SYNC][WRITE]);
	INIT_LIST_HEAD(&nd->queue[ASYNC][READ]);
	INIT_LIST_HEAD(&nd->queue[ASYNC][WRITE]);
	nd->writes_starved = 0;

	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);
	return 0;
}

static struct elevator_type elevator_anxiety = {
	.ops = {
		.elevator_merge_req_fn	= anxiety_merged_requests,
		.elevator_dispatch_fn		= anxiety_dispatch,
		.elevator_add_req_fn		= anxiety_add_request,
		.elevator_former_req_fn	= anxiety_former_request,
		.elevator_latter_req_fn	= anxiety_latter_request,
		.elevator_init_fn				= anxiety_init_queue,
	},
	.elevator_name = "anxiety",
	.elevator_owner = THIS_MODULE,
};

static int __init anxiety_init(void) {
	return elv_register(&elevator_anxiety);
}

static void __exit anxiety_exit(void) {
	elv_unregister(&elevator_anxiety);
}

module_init(anxiety_init);
module_exit(anxiety_exit);

MODULE_AUTHOR("Draco (Tyler Nijmeh)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Anxiety IO scheduler");
