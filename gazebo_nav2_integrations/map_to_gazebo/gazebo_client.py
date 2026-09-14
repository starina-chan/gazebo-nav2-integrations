"""Shared helpers for calling Gazebo Classic's ROS services from a node.

Everything here is chained via call_async()/add_done_callback()/create_timer()
instead of a blocking call or spin_until_future_complete() - either would
deadlock a SingleThreadedExecutor when called from inside a subscription
callback.
"""


def call_with_retry(node, client, request, service_wait_sec, on_done):
    """wait_for_service, then call_async - logging and giving up cleanly
    (calling nothing) if the service never shows up. `on_done(future)` runs
    with the completed future once the call finishes.
    """
    if not client.wait_for_service(timeout_sec=service_wait_sec):
        node.get_logger().warning(
            f"service '{client.srv_name}' unavailable after {service_wait_sec}s "
            f"- is gazebo_ros running?"
        )
        return
    future = client.call_async(request)
    future.add_done_callback(on_done)


def poll_until(node, poll_fn, condition, interval_sec, timeout_sec, on_satisfied, elapsed=0.0):
    """Repeatedly call poll_fn() (-> a future) until condition(response) is
    true or timeout_sec elapses, then call on_satisfied() either way.

    Exists because a service call's success=True only means Gazebo accepted
    the request, not that the world state (e.g. its model list) has caught
    up yet - that update lands on Gazebo's own transport/physics thread some
    variable time later. Used both to confirm a delete actually landed and to
    confirm a spawn actually landed - same wait-and-recheck shape either way,
    just a different `condition`.
    """
    future = poll_fn()

    def _on_response(future):
        try:
            response = future.result()
        except Exception as e:
            node.get_logger().warning(f"poll call raised an exception (giving up): {e}")
            on_satisfied()
            return

        if condition(response):
            on_satisfied()
            return
        if elapsed >= timeout_sec:
            node.get_logger().warning(f"gave up polling after {elapsed:.1f}s - proceeding anyway")
            on_satisfied()
            return

        def _retry():
            timer.cancel()
            node.destroy_timer(timer)
            poll_until(node, poll_fn, condition, interval_sec, timeout_sec, on_satisfied, elapsed + interval_sec)

        timer = node.create_timer(interval_sec, _retry)

    future.add_done_callback(_on_response)
