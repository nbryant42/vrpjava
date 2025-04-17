package com.github.vrpjava.cvrp;

import com.google.errorprone.annotations.concurrent.GuardedBy;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;
import java.util.concurrent.RejectedExecutionException;
import java.util.stream.IntStream;

class Scheduler implements AutoCloseable {
    @GuardedBy("jobs")
    private boolean shuttingDown = false;
    private final List<Job> jobs = new ArrayList<>();
    private final List<Thread> threads;
    private final Worker worker = new Worker(this);

    Scheduler() {
        var threadFactory = Thread.ofPlatform().name("ojAlgoCVRPSolver-", 1).factory();

        threads = IntStream.range(0, Runtime.getRuntime().availableProcessors()).mapToObj(i ->
                threadFactory.newThread(() -> {
                    while (true) {
                        Job job;
                        Node node;

                        synchronized (jobs) {
                            var optional = jobs.stream().filter(Job::hasWork).min(Comparator.comparing(Job::totalTime));
                            if (optional.isEmpty()) {
                                if (shuttingDown && jobs.isEmpty()) {
                                    return;
                                }
                                try {
                                    jobs.wait();
                                } catch (InterruptedException ignored) {
                                }
                                continue;
                            }
                            job = optional.get();
                            node = job.nextNode();
                        }

                        // just using real time instead of thread CPU time to avoid MxBean complexity
                        // and because much of the work happens in other threads (ojAlgo's thread pool)
                        var start = System.nanoTime();
                        worker.process(job, node);
                        job.nodeComplete(System.nanoTime() - start);
                    }
                })
        ).toList();

        threads.forEach(Thread::start);
    }

    void register(Job job) {
        synchronized (jobs) {
            if (shuttingDown) {
                throw new RejectedExecutionException();
            }
            jobs.add(job);
            jobs.forEach(Job::resetTime);
            jobs.notify();
        }
    }

    void deregister(Job job) {
        synchronized (jobs) {
            jobs.remove(job);
            if (shuttingDown && jobs.isEmpty()) {
                jobs.notifyAll();
            }
        }
    }

    /**
     * Initiate an orderly shutdown in which no new {@link Job} instances can be registered, but existing jobs will be
     * allowed to run to completion.
     */
    @Override
    public void close() {
        synchronized (jobs) {
            shuttingDown = true;
            jobs.notifyAll();
        }
        threads.forEach(t -> {
            try {
                t.join();
            } catch (InterruptedException ignored) {
            }
        });
    }

    void queueNodes(Job job, Node... nodes) {
        synchronized (jobs) {
            for (var n : nodes) {
                job.queueNode(n);
                jobs.notify();
            }
        }
    }
}
