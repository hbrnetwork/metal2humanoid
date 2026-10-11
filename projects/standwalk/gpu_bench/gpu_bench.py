"""GPU benchmark suite. Mirrors gpu_experiments_colab.ipynb so the same numbers
can be collected on Colab or on the tower.  Writes results.json next to the script.

Usage:  python gpu_bench.py [--skip-mnist] [--out results.json]
"""
import argparse, itertools, json, platform, subprocess, time
from pathlib import Path

import torch
import torch.nn as nn
import torch.nn.functional as F


def sync():
    if torch.cuda.is_available():
        torch.cuda.synchronize()


def gpu_info():
    assert torch.cuda.is_available(), "No CUDA device visible"
    p = torch.cuda.get_device_properties(0)
    smi = subprocess.run(
        ["nvidia-smi", "--query-gpu=driver_version,temperature.gpu,clocks.sm,clocks.mem,power.draw",
         "--format=csv,noheader"], capture_output=True, text=True).stdout.strip()
    return {
        "host": platform.node(),
        "gpu": p.name,
        "vram_gb": round(p.total_memory / 1e9, 2),
        "sm_count": p.multi_processor_count,
        "compute_capability": f"{p.major}.{p.minor}",
        "cuda_runtime": torch.version.cuda,
        "torch": torch.__version__,
        "python": platform.python_version(),
        "nvidia_smi": smi,
    }


def bench_matmul(dev, n=8192, dtype=torch.float32, iters=20):
    a = torch.randn(n, n, device=dev, dtype=dtype)
    b = torch.randn(n, n, device=dev, dtype=dtype)
    for _ in range(3):
        a @ b
    sync()
    t0 = time.perf_counter()
    for _ in range(iters):
        a @ b
    sync()
    dt = (time.perf_counter() - t0) / iters
    del a, b
    return 2 * n**3 / dt / 1e12


def bench_bandwidth(dev, n_bytes=2 * 1024**3, iters=20):
    x = torch.empty(n_bytes // 4, device=dev, dtype=torch.float32)
    y = torch.empty_like(x)
    for _ in range(3):
        y.copy_(x)
    sync()
    t0 = time.perf_counter()
    for _ in range(iters):
        y.copy_(x)
    sync()
    dt = (time.perf_counter() - t0) / iters
    del x, y
    return 2 * n_bytes / dt / 1e9


class Net(nn.Module):
    def __init__(self):
        super().__init__()
        self.c1 = nn.Conv2d(1, 32, 3)
        self.c2 = nn.Conv2d(32, 64, 3)
        self.f1 = nn.Linear(9216, 128)
        self.f2 = nn.Linear(128, 10)

    def forward(self, x):
        x = F.relu(self.c1(x))
        x = F.max_pool2d(F.relu(self.c2(x)), 2)
        x = torch.flatten(x, 1)
        x = F.relu(self.f1(x))
        return self.f2(x)


def mnist_loaders(root):
    from torchvision import datasets, transforms
    from torch.utils.data import DataLoader
    tfm = transforms.Compose([transforms.ToTensor(), transforms.Normalize((0.1307,), (0.3081,))])
    train = datasets.MNIST(root, train=True, download=True, transform=tfm)
    test = datasets.MNIST(root, train=False, download=True, transform=tfm)
    return (DataLoader(train, batch_size=256, shuffle=True, num_workers=2, pin_memory=True),
            DataLoader(test, batch_size=1000, num_workers=2, pin_memory=True), len(test))


def train_one_epoch(train_dl, test_dl, n_test, device, use_amp=False):
    torch.manual_seed(0)
    model = Net().to(device)
    opt = torch.optim.Adam(model.parameters(), lr=1e-3)
    scaler = torch.amp.GradScaler(enabled=use_amp)
    model.train()
    t0 = time.perf_counter()
    for xb, yb in train_dl:
        xb, yb = xb.to(device, non_blocking=True), yb.to(device, non_blocking=True)
        opt.zero_grad(set_to_none=True)
        with torch.autocast(device_type=device.type, dtype=torch.float16, enabled=use_amp):
            loss = F.cross_entropy(model(xb), yb)
        scaler.scale(loss).backward()
        scaler.step(opt)
        scaler.update()
    sync()
    train_s = time.perf_counter() - t0
    model.eval()
    correct = 0
    with torch.no_grad():
        for xb, yb in test_dl:
            correct += (model(xb.to(device)).argmax(1).cpu() == yb).sum().item()
    return train_s, correct / n_test


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--skip-mnist", action="store_true")
    ap.add_argument("--out", default=str(Path(__file__).with_name("results.json")))
    args = ap.parse_args()

    dev = torch.device("cuda")
    results = {"started": time.strftime("%Y-%m-%d %H:%M:%S"), **gpu_info()}
    print(f"{results['gpu']}  {results['vram_gb']} GB  torch {results['torch']}  cuda {results['cuda_runtime']}")

    results["matmul_tflops"] = {}
    for name, dt in [("fp32", torch.float32), ("tf32", None), ("fp16", torch.float16), ("bf16", torch.bfloat16)]:
        try:
            if name == "tf32":
                torch.backends.cuda.matmul.allow_tf32 = True
                tf = bench_matmul(dev, dtype=torch.float32)
                torch.backends.cuda.matmul.allow_tf32 = False
            else:
                tf = bench_matmul(dev, dtype=dt)
            results["matmul_tflops"][name] = round(tf, 1)
            print(f"matmul {name:5s} {tf:6.1f} TFLOPS")
        except Exception as e:
            results["matmul_tflops"][name] = None
            print(f"matmul {name:5s} unsupported ({type(e).__name__})")
        torch.cuda.empty_cache()

    bw = bench_bandwidth(dev)
    results["mem_bandwidth_gbs"] = round(bw, 0)
    print(f"memory bandwidth {bw:.0f} GB/s")
    torch.cuda.empty_cache()

    if not args.skip_mnist:
        train_dl, test_dl, n_test = mnist_loaders(str(Path(__file__).with_name("data")))
        results["mnist_1epoch"] = {}
        for label, amp in [("gpu_fp32", False), ("gpu_amp_fp16", True)]:
            s, acc = train_one_epoch(train_dl, test_dl, n_test, dev, amp)
            results["mnist_1epoch"][label] = {"train_seconds": round(s, 1), "test_acc": round(acc, 4)}
            print(f"mnist {label:13s} {s:6.1f}s  acc={acc:.4f}")
        torch.manual_seed(0)
        m, o = Net(), None
        o = torch.optim.Adam(m.parameters())
        n_batches = 20
        t0 = time.perf_counter()
        for xb, yb in itertools.islice(train_dl, n_batches):
            o.zero_grad()
            F.cross_entropy(m(xb), yb).backward()
            o.step()
        cpu_est = (time.perf_counter() - t0) / n_batches * len(train_dl)
        results["mnist_1epoch"]["cpu_fp32_estimated"] = {"train_seconds": round(cpu_est, 1)}
        print(f"mnist cpu_fp32 (est) {cpu_est:6.1f}s")

    results["peak_vram_gb"] = round(torch.cuda.max_memory_allocated() / 1e9, 2)
    results["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
    Path(args.out).write_text(json.dumps(results, indent=2))
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
