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

/* default tunable values */
static const unsigned int max_writes_starved = 8; /* max amount of times reads can starve pending writes */

struct anxiety_data {
	struct list_head queue[2];
	unsigned int writes_starved;

	/* tunables */
	unsigned int max_writes_starved;
};

static void anxiety_merged_requests(struct request_queue *q, struct request *rq, struct request *next)
{
	rq_fifo_clear(next);
}

static __always_inline struct request *anxiety_choose_request(struct anxiety_data *mdata)
{
	/* prioritize reads unless writes are exceedingly starved */
	bool starved = mdata->writes_starved > mdata->max_writes_starved;

	/* read */
	if (!starved && !list_empty(&mdata->queue[READ])) {
		mdata->writes_starved++;
		return rq_entry_fifo(mdata->queue[READ].next);
	}

	/* write */
	if (!list_empty(&mdata->queue[WRITE])) {
		mdata->writes_starved = 0;
		return rq_entry_fifo(mdata->queue[WRITE].next);
	}

	/* all queues are empty, i.e. no pending requests */
	mdata->writes_starved = 0;
	return NULL;
}

static int anxiety_dispatch(struct request_queue *q, int force)
{
	struct request *rq = anxiety_choose_request(q->elevator->elevator_data);

	if (!rq)
		return 0;

	rq_fifo_clear(rq);
	elv_dispatch_add_tail(rq->q, rq);

	return 1;
}

static void anxiety_add_request(struct request_queue *q, struct request *rq)
{
	const uint8_t dir = rq_data_dir(rq);

	list_add_tail(&rq->queuelist, &((struct anxiety_data *) q->elevator->elevator_data)->queue[dir]);
}

static struct request *anxiety_former_request(struct request_queue *q, struct request *rq)
{
	const uint8_t dir = rq_data_dir(rq);

	if (rq->queuelist.prev == &((struct anxiety_data *) q->elevator->elevator_data)->queue[dir])
		return NULL;

	return list_prev_entry(rq, queuelist);
}

static struct request *anxiety_latter_request(struct request_queue *q, struct request *rq)
{
	const uint8_t dir = rq_data_dir(rq);

	if (rq->queuelist.next == &((struct anxiety_data *) q->elevator->elevator_data)->queue[dir])
		return NULL;

	return list_next_entry(rq, queuelist);
}

static int anxiety_init_queue(struct request_queue *q, struct elevator_type *elv)
{
	struct anxiety_data *data;
	struct elevator_queue *eq = elevator_alloc(q, elv);

	if (!eq)
		return -ENOMEM;

	/* allocate data */
	data = kmalloc_node(sizeof(*data), GFP_KERNEL, q->node);
	if (!data) {
		kobject_put(&eq->kobj);
		return -ENOMEM;
	}
	eq->elevator_data = data;

	/* initialize data */
	INIT_LIST_HEAD(&data->queue[READ]);
	INIT_LIST_HEAD(&data->queue[WRITE]);
	data->writes_starved = 0;
	data->max_writes_starved = max_writes_starved;

	/* set the elevator to us */
	spin_lock_irq(q->queue_lock);
	q->elevator = eq;
	spin_unlock_irq(q->queue_lock);

	return 0;
}

/* sysfs tunables */
static ssize_t anxiety_max_writes_starved_show(struct elevator_queue *e, char *page)
{
	struct anxiety_data *ad = e->elevator_data;

	return snprintf(page, PAGE_SIZE, "%d\n", ad->max_writes_starved);
}

static ssize_t anxiety_max_writes_starved_store(struct elevator_queue *e, const char *page, size_t count)
{
	struct anxiety_data *ad = e->elevator_data;
	int ret;

	ret = kstrtouint(page, 0, &ad->max_writes_starved);
	if (ret < 0)
		return ret;

	return count;
}

static struct elv_fs_entry anxiety_attrs[] = {
	__ATTR(max_writes_starved, 0644, anxiety_max_writes_starved_show, anxiety_max_writes_starved_store),
	__ATTR_NULL
};

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
	.elevator_attrs = anxiety_attrs,
	.elevator_owner = THIS_MODULE,
};

static int __init anxiety_init(void)
{
	return elv_register(&elevator_anxiety);
}

static void __exit anxiety_exit(void)
{
	elv_unregister(&elevator_anxiety);
}

module_init(anxiety_init);
module_exit(anxiety_exit);

MODULE_AUTHOR("Draco (Tyler Nijmeh)");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Anxiety IO scheduler");
