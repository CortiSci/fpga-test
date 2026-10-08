// Windows regression: a configured read timeout must also cover partial frames.
#include "../../Software Emulator/shim/FTD3XX_shim.cpp"
#include <iostream>

int main() {
    EmuHandle h;
    const std::string name = "\\\\.\\pipe\\IONM_TIMEOUT_TEST_" + std::to_string(GetCurrentProcessId());
    HANDLE writer = CreateNamedPipeA(name.c_str(), PIPE_ACCESS_OUTBOUND,
        PIPE_TYPE_BYTE | PIPE_WAIT, 1, 4096, 4096, 0, nullptr);
    h.data_pipe = CreateFileA(name.c_str(), GENERIC_READ, 0, nullptr, OPEN_EXISTING,
                              FILE_FLAG_OVERLAPPED, nullptr);
    if (writer == INVALID_HANDLE_VALUE || h.data_pipe == INVALID_HANDLE_VALUE) return 2;
    if (!ConnectNamedPipe(writer, nullptr) && GetLastError() != ERROR_PIPE_CONNECTED) return 2;
    FT_SetPipeTimeout(&h, 0x82, 40);
    int failures = 0;
    auto timeout = [&](const char* name) {
        std::mutex mu;
        std::condition_variable cv;
        bool finished = false;
        std::thread guard([&] {
            std::unique_lock<std::mutex> lock(mu);
            if (!cv.wait_for(lock, std::chrono::milliseconds(400), [&]{ return finished; }))
                FT_AbortPipe(&h, 0x82);
        });
        UCHAR buf[8]; ULONG count = 99;
        auto start = std::chrono::steady_clock::now();
        auto status = FT_ReadPipe(&h, 0x82, buf, sizeof(buf), &count, nullptr);
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
        { std::lock_guard<std::mutex> lock(mu); finished = true; }
        cv.notify_one(); guard.join();
        bool ok = status == FT_TIMEOUT && count == 0 && ms >= 25 && ms < 250;
        std::cout << (ok ? "PASS " : "FAIL ") << name << " status=" << status << " ms=" << ms << '\n';
        failures += !ok;
    };
    timeout("idle");
    const unsigned char packet[] = {8, 0, 1, 2, 3, 4, 5, 6, 7, 8};
    write_all(writer, packet, 1);
    timeout("partial prefix");
    write_all(writer, packet+1, 4);
    timeout("partial payload");
    write_all(writer, packet+5, 5);
    UCHAR buf[8]; ULONG count = 0;
    auto status = FT_ReadPipe(&h, 0x82, buf, 8, &count, nullptr);
    bool ok = status == FT_OK && count == 8 && !memcmp(buf, packet+2, 8);
    std::cout << (ok ? "PASS" : "FAIL") << " recovery preserves frame\n";
    failures += !ok;
    h.mode2_enabled = true;
    timeout("buffered idle");
    FT_SetPipeTimeout(&h, 0x82, 2000);
    std::thread aborter([&] { Sleep(40); FT_AbortPipe(&h, 0x82); });
    auto start = std::chrono::steady_clock::now();
    status = FT_ReadPipe(&h, 0x82, buf, 8, &count, nullptr);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
    aborter.join();
    ok = status == FT_IO_ERROR && ms < 250;
    std::cout << (ok ? "PASS " : "FAIL ") << "buffered abort ms=" << ms << '\n';
    failures += !ok;
    h.m2.accum.assign(packet+2, packet+10);
    status = FT_ReadPipe(&h, 0x82, buf, 8, &count, nullptr);
    ok = status == FT_OK && count == 8 && !memcmp(buf, packet+2, 8);
    std::cout << (ok ? "PASS" : "FAIL") << " buffered recovery\n";
    failures += !ok;
    h.m2.feeder_done = true;
    start = std::chrono::steady_clock::now();
    status = FT_ReadPipe(&h, 0x82, buf, 8, &count, nullptr);
    ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-start).count();
    ok = status == FT_FAILED_TO_READ_DEVICE && ms < 250;
    std::cout << (ok ? "PASS" : "FAIL") << " buffered disconnect\n";
    failures += !ok;
    CloseHandle(writer); CloseHandle(h.data_pipe);
    return failures ? 1 : 0;
}
