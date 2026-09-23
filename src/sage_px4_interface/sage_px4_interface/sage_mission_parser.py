#!/usr/bin/env python3
"""Step 19 (stage 1): rule-based mission parser.

Turns a natural-language sentence into a structured mission spec:

    {target_class, attribute, quantity, action, output,
     valid, supported, reason, text}

/sage/mission/command (std_msgs/String)  ->  /sage/mission/spec
(std_msgs/String holding JSON, transient-local so late subscribers
still get the latest mission).

Stage 2 (LLM) will produce the same spec; everything downstream stays
unchanged.
"""

import json
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy

from std_msgs.msg import String


# Classes the perception stack can actually detect today
# (yolo_detector.py is hard-wired to COCO class 0).
SUPPORTED_CLASSES = {'person'}

ACTIONS = {
    'find': 'search', 'search': 'search', 'locate': 'search',
    'look': 'search', 'detect': 'search', 'spot': 'search',
    'scan': 'search',
}

# word -> canonical class
CLASS_WORDS = {
    'person': 'person', 'people': 'person', 'persons': 'person',
    'human': 'person', 'humans': 'person', 'man': 'person',
    'men': 'person', 'woman': 'person', 'women': 'person',
    'victim': 'person', 'victims': 'person',
    'survivor': 'person', 'survivors': 'person',
    'pedestrian': 'person', 'pedestrians': 'person',
    'car': 'car', 'cars': 'car', 'vehicle': 'car', 'vehicles': 'car',
    'truck': 'truck', 'trucks': 'truck', 'bus': 'bus', 'buses': 'bus',
    'bicycle': 'bicycle', 'bicycles': 'bicycle', 'bike': 'bicycle',
    'bikes': 'bicycle', 'motorcycle': 'motorcycle',
    'motorcycles': 'motorcycle', 'dog': 'dog', 'dogs': 'dog',
    'boat': 'boat', 'boats': 'boat',
}

PLURAL_WORDS = {
    'people', 'persons', 'humans', 'men', 'women', 'victims',
    'survivors', 'pedestrians', 'cars', 'vehicles', 'trucks', 'buses',
    'bicycles', 'bikes', 'motorcycles', 'dogs', 'boats',
}

COLORS = {
    'red', 'blue', 'green', 'yellow', 'white', 'black', 'orange',
    'grey', 'gray', 'purple', 'pink', 'brown',
}

NUMBER_WORDS = {
    'one': 1, 'two': 2, 'three': 3, 'four': 4, 'five': 5,
    'six': 6, 'seven': 7, 'eight': 8, 'nine': 9, 'ten': 10,
}

ALL_WORDS = {'all', 'every', 'each', 'everyone', 'everybody'}
OUTPUT_WORDS = {
    'report': 'locations', 'locations': 'locations',
    'location': 'locations', 'where': 'locations',
    'positions': 'locations', 'position': 'locations',
    'count': 'count', 'many': 'count', 'number': 'count',
}


def parse_mission(text):
    """Parse a mission sentence. Pure function (no ROS)."""

    spec = {
        'text': text,
        'action': None,
        'target_class': None,
        'attribute': None,
        'quantity': None,
        'output': 'locations',
        'valid': False,
        'supported': False,
        'reason': '',
    }

    words = re.findall(r"[a-z0-9]+", text.lower())

    if not words:
        spec['reason'] = 'empty command'
        return spec

    # Action: first recognised verb.
    for w in words:
        if w in ACTIONS:
            spec['action'] = ACTIONS[w]
            break

    if spec['action'] is None:
        spec['reason'] = 'no supported action (try "find ...")'
        return spec

    # Target class: first recognised class word.
    target_word = None
    for w in words:
        if w in CLASS_WORDS:
            spec['target_class'] = CLASS_WORDS[w]
            target_word = w
            break

    if spec['target_class'] is None:
        spec['reason'] = 'no known target class in command'
        return spec

    # Attribute: a colour word.
    for w in words:
        if w in COLORS:
            spec['attribute'] = 'gray' if w == 'grey' else w
            break

    # Quantity.
    quantity = None
    for w in words:
        if w in ALL_WORDS:
            quantity = 'all'
            break
        if w in NUMBER_WORDS:
            quantity = NUMBER_WORDS[w]
            break
        if w.isdigit() and int(w) > 0:
            quantity = int(w)
            break

    if quantity is None:
        if 'first' in words or 'any' in words:
            quantity = 1
        elif target_word in PLURAL_WORDS:
            quantity = 'all'
        else:
            quantity = 1

    spec['quantity'] = quantity

    # Output.
    for w in words:
        if w in OUTPUT_WORDS:
            spec['output'] = OUTPUT_WORDS[w]
            break

    spec['valid'] = True
    spec['supported'] = spec['target_class'] in SUPPORTED_CLASSES

    if not spec['supported']:
        spec['reason'] = (
            f"class '{spec['target_class']}' is not supported by "
            'perception yet (supported: '
            f"{', '.join(sorted(SUPPORTED_CLASSES))})"
        )

    return spec


class SageMissionParser(Node):

    def __init__(self):
        super().__init__('sage_mission_parser')

        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.command_sub = self.create_subscription(
            String,
            '/sage/mission/command',
            self.command_callback,
            10
        )

        self.spec_pub = self.create_publisher(
            String,
            '/sage/mission/spec',
            latched
        )

        # Imported lazily: sage_mission_llm imports this module.
        from sage_px4_interface import sage_mission_llm
        self.llm = sage_mission_llm

        self.declare_parameter('use_llm', True)
        self.llm_parse = None
        backend = 'rule-based only'

        if self.get_parameter('use_llm').value:
            found = self.llm.make_parser()
            if found:
                backend, self.llm_parse = found
                backend = f'{backend} LLM with rule-based fallback'

        mode = backend
        self.get_logger().info(
            f'SAGE mission parser started ({mode}). '
            'Send text on /sage/mission/command.'
        )

    def command_callback(self, msg):
        spec = None

        if self.llm_parse is not None:
            try:
                spec = self.llm_parse(msg.data)
                self.get_logger().info(f"Mission understood by LLM ({spec.get('source')}).")
            except self.llm.LLMParseError as e:
                self.get_logger().warn(
                    f'LLM parse failed ({e}); using rule-based parser.'
                )

        if spec is None:
            spec = parse_mission(msg.data)
            spec['source'] = 'rules'

        out = String()
        out.data = json.dumps(spec)
        self.spec_pub.publish(out)

        if spec['valid'] and spec['supported']:
            self.get_logger().info(
                'MISSION PARSED | '
                f"action={spec['action']} | "
                f"target={spec['target_class']} | "
                f"attribute={spec['attribute']} | "
                f"quantity={spec['quantity']} | "
                f"output={spec['output']}"
            )
        else:
            self.get_logger().warn(
                f"MISSION NOT EXECUTABLE | text='{msg.data}' | "
                f"reason={spec['reason']}"
            )


def main(args=None):
    rclpy.init(args=args)

    node = SageMissionParser()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
