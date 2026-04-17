# as75
Toàn bộ chương trình này được tạo ra với mục tiêu điều khiển cho nồi hấp thiết bị y tế 75L bằng STM32F407

Tác giả: Vũ Nam Hưng aka Karukosa

## Bảng vận hành + lỗi theo giai đoạn 

| Giai đoạn | Hành động chình	                               | Lỗi có thể xuất hiện         | Hành vi khi lỗi                               |

| Khởi động | Kiểm tra điều khiển trước khi vận hành         | Er01, Er02                   | Giữ ở trạng thái lỗi, chưa cho chạy chu trình.|
| Chờ chạy  | Chọn chương trình, bấm START để vào run        | Er02, Er03                   | Không vào RUN cho đến khi điều kiện đạt.      |
| Vacuum    | Hút chân không + gia nhiệt xen kẽ theo chu kỳ  | Er01, Er02, Er03, Er05       | Dừng an toàn ngay.                            |     
| Heat      | Gia nhiệt đến chế độ cài đặt                   | Er01, Er02, Er03, Er04, Er05 | Dừng an toàn ngay.                            |
| Hold      | Giữ nhiệt bằng PID trong thời gian tiệt trùng  | Er01, Er02, Er03, Er05.      | Dừng an toàn ngay.                            | 
| Vent      | Xả nước, xả khí, hút chân không lại            | Er01, Er03, Er05.            | Dừng an toàn ngay.                            |
| Dry       | Sấy theo thời gian, giữ nhiệt ngưỡng nhiệt sấy | Er01, Er03, Er05.            | Dừng an toàn ngay.                            |    
| Hoàn tất  | Báo hoàn thành và về Chờ chạy                  | x                            | x                                             |

## Kiểm tra logic Valve 5 (mồi nước cho bơm)

Trong firmware hiện tại, `Valve 5` (PB12) được điều khiển để phục vụ mồi nước theo 2 cơ chế:

1. **Mồi sau khi bơm dừng ở cuối chu trình**
   - Sau khi chương trình chạy xong, hệ thống chờ `5 giây`.
   - Sau đó mở `Valve 5` trong `5 giây` để mồi nước.
2. **Mồi xung trong giai đoạn sấy (Dry)**
   - Nếu thời gian sấy lớn hơn `15 phút`, mở xung `Valve 5` ở mốc `30%` và `60%` thời gian sấy (mỗi xung `3 giây`).
   - Nếu thời gian sấy lớn hơn `40 phút`, thêm một xung ở mốc `85%` thời gian sấy (cũng `3 giây`).

Các ngưỡng đang dùng trong mã nguồn:
- `VALVE5_POST_RUN_DELAY_MS = 5000`
- `VALVE5_POST_RUN_PRIME_MS = 5000`
- `VALVE5_DRY_PRIME_PULSE_MS = 3000`
- `VALVE5_DRY_MIN_THRESHOLD_MS = 15 * 60000`
- `VALVE5_DRY_EXTENDED_THRESHOLD_MS = 40 * 60000`
