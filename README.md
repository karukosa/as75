# as75
Toàn bộ chương trình này được tạo ra với mục tiêu điều khiển cho nồi hấp thiết bị y tế 75L bằng STM32F407

Tác giả: Vũ Nam Hưng aka Karukosa

## Bảng vận hành + lỗi theo giai đoạn 

| Giai đoạn | Hành động chình	                               | Lỗi có thể xuất hiện         | Hành vi khi lỗi | 
================================================================================================================
| Khởi động | Kiểm tra điều khiển trước khi vận hành         | Er01, Er02                   | Giữ ở trạng thái lỗi, chưa cho chạy chu trình.
| Chờ chạy  | Chọn chương trình, bấm START để vào run        | Er02, Er03                   | Không vào RUN cho đến khi điều kiện đạt.
| Vacuum    | Hút chân không + gia nhiệt xen kẽ theo chu kỳ  | Er01, Er02, Er03, Er05       | Dừng an toàn ngay.
| Heat      | Gia nhiệt đến chế độ cài đặt                   | Er01, Er02, Er03, Er04, Er05 | Dừng an toàn ngay.
| Hold      | Giữ nhiệt bằng PID trong thời gian tiệt trùng  | Er01, Er02, Er03, Er05.      | Dừng an toàn ngay.
| Vent      | Xả nước, xả khí, hút chân không lại            | Er01, Er03, Er05.            | Dừng an toàn ngay.
| Dry       | Sấy theo thời gian, giữ nhiệt ngưỡng nhiệt sấy | Er01, Er03, Er05.            | Dừng an toàn ngay.
| Hoàn tất  | Báo hoàn thành và về Chờ chạy                  | x                            | x

