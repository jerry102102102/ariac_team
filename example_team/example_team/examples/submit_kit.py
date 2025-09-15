import asyncio

import rclpy
from rclpy.time import Duration
from rclpy.executors import MultiThreadedExecutor

from example_team.environment_interface import Environment
from example_team.utils import AsyncUtils

from ariac_interfaces.msg import AgvStations

async def run():
    rclpy.init()

    # Create and spin executor
    executor = MultiThreadedExecutor()
    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(AsyncUtils.spin_executor(executor, shutdown_event))

    environment = Environment()
    executor.add_node(environment)

    # Create robot nodes

    try:
        # Wait for competition to be ready
        environment.get_logger().info("Waiting for environment to be ready")
        await environment.competition.ready.wait()

        environment.get_logger().info("Starting competition")
        await environment.competition.start()

        environment.get_logger().info("Moving AGV1 to shipping")
        await environment.agvs[1].move(AgvStations.SHIPPING)

        environment.get_logger().info("Submitting kitting order")
        await environment.competition.submit_kit()
        
        await AsyncUtils.await_for_duration(environment.get_clock(), Duration(seconds=2))

        environment.get_logger().info("Ending competition")
        await environment.competition.end(shutdown=True)

    except Exception as e:
        print(5*'\n', e, 5*'\n')
    finally:
        shutdown_event.set()
        await spin_task  

def main():
    asyncio.run(run())