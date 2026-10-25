#!/usr/bin/env python3
"""Publish a retained v1 JSON interface snapshot and wait for the bridge ACK.

This simulates the configuration part of a device; Rosnect must implement the
same exchange itself for automatic updates from its UI. Requires python3-paho-mqtt.
"""
import argparse
import json
import os
from pathlib import Path
import re
import sys
import threading
import uuid

import paho.mqtt.client as mqtt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", type=Path, help="BridgeConfig JSON file with configuration_id/schema_version")
    parser.add_argument("--broker", required=True)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--base-topic", default="ros2embedded")
    parser.add_argument("--timeout", type=float, default=15)
    parser.add_argument("--username")
    parser.add_argument("--password-env", help="Environment variable containing the broker password")
    parser.add_argument("--tls", action="store_true")
    parser.add_argument("--ca-file")
    parser.add_argument("--client-cert")
    parser.add_argument("--client-key")
    args = parser.parse_args()
    config = json.loads(args.config.read_text())
    if not isinstance(config, dict):
        parser.error("configuration must be a JSON object")
    if args.port is not None and not 1 <= args.port <= 65535:
        parser.error("port must be between 1 and 65535")
    connection_id = config.get("connection_id", "")
    config_id = config.get("configuration_id", "")
    if config.get("schema_version") != 1 or not all(
            isinstance(value, str) and re.fullmatch(r"[A-Za-z0-9_.-]{1,96}", value)
            for value in (connection_id, config_id)):
        parser.error("schema_version=1 and valid connection_id/configuration_id are required")
    domain = config.get("domain_id")
    if type(domain) is not int or not 0 <= domain <= 0xFFFFFFFF:
        parser.error("domain_id must fit uint32")
    if not args.base_topic or any(c in args.base_topic for c in ("+", "#", "\0")):
        parser.error("invalid MQTT base topic")
    if bool(args.client_cert) != bool(args.client_key):
        parser.error("both client certificate and key are required")
    if not args.tls and any((args.ca_file, args.client_cert, args.client_key)):
        parser.error("certificate options require --tls")
    if args.password_env and (not args.username or args.password_env not in os.environ):
        parser.error("--password-env requires --username and an existing environment variable")
    if args.timeout <= 0:
        parser.error("timeout must be positive")
    wire = json.dumps(config, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
    if len(wire) > 65536:
        parser.error("configuration exceeds 64 KiB")
    prefix = f"{args.base_topic}/{domain}/{connection_id}"
    completed = threading.Event()
    result = []
    client = mqtt.Client(client_id="ros2embedded-config-" + uuid.uuid4().hex, protocol=mqtt.MQTTv311)
    if args.username:
        client.username_pw_set(args.username, os.environ[args.password_env] if args.password_env else None)
    if args.tls:
        client.tls_set(ca_certs=args.ca_file, certfile=args.client_cert, keyfile=args.client_key)

    def connected(client, userdata, flags, rc):
        if rc != 0:
            result.append({"accepted": False, "error": f"Broker rejected connection: {rc}"})
            completed.set()
        else:
            client.subscribe(prefix + "/config_status", qos=1)

    def subscribed(client, userdata, mid, granted_qos):
        if not granted_qos or granted_qos[0] == 128:
            result.append({"accepted": False, "error": "Broker denied config_status subscription"})
            completed.set()
            return
        client.publish(prefix + "/config", wire, qos=1, retain=True)

    def message(client, userdata, msg):
        try:
            status = json.loads(msg.payload)
            if (status.get("connection_id"), status.get("domain_id"), status.get("configuration_id")) == (
                    connection_id, domain, config_id):
                result.append(status)
                completed.set()
        except (ValueError, TypeError, AttributeError):
            pass

    client.on_connect, client.on_subscribe, client.on_message = connected, subscribed, message
    try:
        client.connect(args.broker, args.port or (8883 if args.tls else 1883), 15)
        client.loop_start()
        if not completed.wait(args.timeout):
            print("No matching bridge acknowledgement. Check bridge identity, config_source=device, and config_encoding=json.", file=sys.stderr)
            return 2
        print(json.dumps(result[-1], indent=2))
        return 0 if result[-1].get("accepted") is True else 1
    finally:
        client.disconnect()
        client.loop_stop()


if __name__ == "__main__":
    sys.exit(main())
