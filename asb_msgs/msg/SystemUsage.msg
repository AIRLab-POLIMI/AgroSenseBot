float32[] cpu_core_usage_percent  # [%] per core usage
float64 mem_total  # [MiB] total physical memory (exclusive swap)
float64 mem_available  # [MiB] the memory that can be given instantly to processes without the system going into swap
float64 mem_used  # [MiB] memory used, the sum of used and available does not necessarily equal total
float32 mem_used_percent  # [%] the percentage usage calculated as (total - available) / total * 100
asb_msgs/ProcessCpuUsage[] processes
