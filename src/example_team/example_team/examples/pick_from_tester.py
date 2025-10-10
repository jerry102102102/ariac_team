import asyncio

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Duration

from example_team.environment_interface import Environment
from example_team.robots import InspectionRobot2
from example_team.utils import AsyncUtils

async def run():
    rclpy.init()

    # Create and spin executor
    executor = MultiThreadedExecutor()
    shutdown_event = asyncio.Event()
    spin_task = asyncio.create_task(AsyncUtils.spin_executor(executor, shutdown_event))

    environment = Environment()
    executor.add_node(environment)

    # Create robot nodes
    ir2 = InspectionRobot2()
    executor.add_node(ir2)

    try:
        # Wait for competition to be ready
        environment.get_logger().info("Waiting for environment to be ready")
        await environment.competition.ready.wait()

        await ir2.ready()

        for i in (1, 2):
            await ir2.pick_cell_from_tester(i)
            await ir2.recycle_cell()

    except Exception as e:
        print(5*'\n', e, 5*'\n')
    finally:
        shutdown_event.set()
        await spin_task  

def main():
    asyncio.run(run())