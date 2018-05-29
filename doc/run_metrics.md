# Metrics Pipeline

This document briefly describes how to run the metrics pipeline to
collect and show information about the system.

The pipeline has the following components:
- [Prometheus](https://prometheus.io/):
  Provides the collector and time-series database for metrics
  information.
- [node_exporter](https://github.com/prometheus/node_exporter)
  Extracts and exports information about a specific compute node,
  i.e., our robot host system.
- Fawkes Metrics system
  Provides Fawkes-specific metrics.
- [Grafana](https://grafana.com/)
  Visualization tool for data extracted from Prometheus.

We run the external tools from Docker images, as it makes integration
very easy. We also use the host network to avoid tricky network
configuration. The robot firewall can regulate acceptable external
access.

We use an environment variable $FAWKES_DIR to refer to your local
Fawkes directory. Set or modify as necessary.

## Run Collector Pipeline
Fawkes comes with a useful default configuration. It assumes a
single-host setup, i.e., it defines a few static scrape targets (from
which data is retrieved):
- Prometheus information itself
- Information from node_exporter
- Information from Fawkes

Since we are sensitive to time sync, we enable the NTP collector. This
requires allowing SNTP access from chrony:

```bash
  echo "allow 127.0.0.1" >> /etc/chrony.conf
```

```bash
docker run --name prometheus -d --net=host \
  -v $FAWKES_DIR/cfg/prometheus:/etc/prometheus/ \
  quay.io/prometheus/prometheus

docker run --name node_exporter -d --net=host --pid=host \
  quay.io/prometheus/node-exporter --collector.ntp

docker run --name process-exporter -d --privileged \
  -v /proc:/host/proc -p 127.0.0.1:9256:9256 \
  timn/process-exporter:latest -procfs /host/proc \
  -procnames prometheus,grafana-server,fawkes,process-exporter \
  -threads
```

## Visualization
Basic visualization is integrated into Webview 2.0. For a wider array
of visulaization we use Grafana 5. We use its provisioning feature to
install default datasources and dashboards.

```bash
docker run --name=grafana -d --net=host \
  -v $FAWKES_DIR/cfg/grafana:/etc/grafana grafana/grafana
```

