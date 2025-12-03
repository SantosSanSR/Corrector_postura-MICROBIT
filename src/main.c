/* Corrector de Postura - Proyecto Final */

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/display/mb_display.h>
#include <zephyr/logging/log.h>
#include <math.h>

LOG_MODULE_REGISTER(PostureCorrector, LOG_LEVEL_INF);

/* ==================== BLE CONFIGURATION ==================== */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12)
};

static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_le_adv_param adv_params = {
    .options = BT_LE_ADV_OPT_CONN,
    .interval_min = BT_GAP_ADV_FAST_INT_MIN_2,
    .interval_max = BT_GAP_ADV_FAST_INT_MAX_2,
};

/* ==================== BLE DATA VARIABLES ==================== */
static int8_t temperature_value = 0;
static struct bt_conn *current_conn;

/* ==================== BLE CALLBACKS ==================== */
static ssize_t read_temperature(struct bt_conn *conn,
                                const struct bt_gatt_attr *attr,
                                void *buf, uint16_t len, uint16_t offset)
{
    return bt_gatt_attr_read(conn, attr, buf, len, offset,
                             &temperature_value, sizeof(temperature_value));
}

static void ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value) {
    LOG_INF("Notificaciones %s",
            value == BT_GATT_CCC_NOTIFY ? "activadas" : "desactivadas");
}

static void connected(struct bt_conn *conn, uint8_t err) {
    if (err) {
        LOG_ERR("Error de conexión: %u", err);
    } else {
        current_conn = bt_conn_ref(conn);
        LOG_INF("Conectado");
    }
}

static void disconnected(struct bt_conn *conn, uint8_t reason) {
    LOG_INF("Desconectado (razón: %u)", reason);
    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }
    int ret = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad),
                              sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Error reiniciando advertising: %d", ret);
    }
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

/* ==================== GATT SERVICE ==================== */
BT_GATT_SERVICE_DEFINE(posture_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_128(
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12)),
   
    // Característica de temperatura (READ y NOTIFY)
    BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_128(
        0xf1, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12,
        0xf0, 0xde, 0xbc, 0x9a, 0x78, 0x56, 0x34, 0x12),
        BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,
        BT_GATT_PERM_READ,
        read_temperature, NULL, &temperature_value),
    BT_GATT_CCC(ccc_cfg_changed,
                BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
);

/* ==================== DISPLAY IMAGES ==================== */
// Punto en el centro para calibración
static const struct mb_image IMG_CALIBRATE = MB_IMAGE({0,0,0,0,0},
                                                      {0,0,0,0,0},
                                                      {0,0,1,0,0},
                                                      {0,0,0,0,0},
                                                      {0,0,0,0,0});

// Display vacío (apagado)
static const struct mb_image IMG_EMPTY = MB_IMAGE({0,0,0,0,0},
                                                  {0,0,0,0,0},
                                                  {0,0,0,0,0},
                                                  {0,0,0,0,0},
                                                  {0,0,0,0,0});

// Dos líneas verticales (II) para postura correcta
static const struct mb_image IMG_CORRECT = MB_IMAGE({0,1,0,1,0},
                                                    {0,1,0,1,0},
                                                    {0,1,0,1,0},
                                                    {0,1,0,1,0},
                                                    {0,1,0,1,0});

// <> para inclinado hacia adelante
static const struct mb_image IMG_FORWARD = MB_IMAGE({1,0,0,0,1},
                                                    {0,1,0,1,0},
                                                    {0,0,1,0,0},
                                                    {0,1,0,1,0},
                                                    {1,0,0,0,1});

// >< para inclinado hacia atrás
static const struct mb_image IMG_BACKWARD = MB_IMAGE({0,0,1,0,0},
                                                     {0,1,0,1,0},
                                                     {1,0,0,0,1},
                                                     {0,1,0,1,0},
                                                     {0,0,1,0,0});

/* ==================== HARDWARE DEFINITIONS ==================== */
#define BUTTON_A_NODE DT_ALIAS(sw0)
static const struct gpio_dt_spec button_a = GPIO_DT_SPEC_GET(BUTTON_A_NODE, gpios);
static struct gpio_callback button_a_cb;

#define ACCEL_NODE DT_ALIAS(accel0)
static const struct device *const accel_dev = DEVICE_DT_GET(ACCEL_NODE);

static const struct device *const temp_dev = DEVICE_DT_GET_ANY(nordic_nrf_temp);
static struct mb_display *display;

/* ==================== POSTURE VARIABLES ==================== */
static float reference_z = 0.0;
static const float THRESHOLD = 1.2;  // Aumentado de 0.5 a 1.2
static bool calibrated = false;
static bool calibrating = false;
static uint32_t last_blink_time = 0;
static bool blink_state = true;
static bool alert_blinking = false;
static uint32_t last_alert_blink_time = 0;
static bool alert_blink_state = true;

/* ==================== FUNCTION PROTOTYPES ==================== */
static void button_a_pressed(const struct device *dev,
                             struct gpio_callback *cb, uint32_t pins);
static void read_accelerometer(float *x, float *y, float *z);
static void update_display(void);
static void send_temperature_notification(void);

/* ==================== MAIN APPLICATION ==================== */
int main(void) {
    int ret;
   
    LOG_INF("Iniciando Corrector de Postura...");
   
    // 1. Inicializar display
    display = mb_display_get();
    if (display == NULL) {
        LOG_ERR("Error obteniendo display");
        return 0;
    }
   
    // Mostrar mensaje inicial
    mb_display_print(display, MB_DISPLAY_MODE_SCROLL, 500, "POSTURA");
    k_msleep(1500);
   
    // 2. Inicializar botón A
    if (!device_is_ready(button_a.port)) {
        LOG_ERR("Botón A no disponible");
        return 0;
    }
   
    ret = gpio_pin_configure_dt(&button_a, GPIO_INPUT | GPIO_PULL_UP);
    if (ret < 0) {
        LOG_ERR("Error configurando botón A: %d", ret);
        return 0;
    }
   
    gpio_init_callback(&button_a_cb, button_a_pressed, BIT(button_a.pin));
    gpio_add_callback(button_a.port, &button_a_cb);
    gpio_pin_interrupt_configure_dt(&button_a, GPIO_INT_EDGE_TO_ACTIVE);
   
    // 3. Verificar sensores
    if (!device_is_ready(accel_dev)) {
        LOG_ERR("Acelerómetro no disponible");
        mb_display_print(display, MB_DISPLAY_MODE_SCROLL, 500, "NO ACEL");
        return 0;
    }
   
    if (!device_is_ready(temp_dev)) {
        LOG_ERR("Sensor de temperatura no disponible");
    }
   
    // 4. Inicializar BLE
    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("Error iniciando BLE: %d", ret);
        return 0;
    }
   
    ret = bt_le_adv_start(&adv_params, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Error iniciando advertising: %d", ret);
        return 0;
    }
   
    LOG_INF("Dispositivo listo: %s", CONFIG_BT_DEVICE_NAME);
    LOG_INF("Presione el botón A para calibrar postura correcta");
   
    // 5. Bucle principal
    uint32_t last_temp_update = 0;
    uint32_t last_display_update = 0;
   
    while (1) {
        uint32_t current_time = k_uptime_get();
       
        // Actualizar display
        if (current_time - last_display_update > 100) {
            update_display();
            last_display_update = current_time;
        }
       
        // Enviar temperatura cada 2 segundos
        if (current_time - last_temp_update > 2000) {
            send_temperature_notification();
            last_temp_update = current_time;
        }
       
        k_msleep(50);
    }
   
    return 0;
}

/* ==================== ACCELEROMETER READING ==================== */
static void read_accelerometer(float *x, float *y, float *z) {
    struct sensor_value accel_x, accel_y, accel_z;
   
    if (sensor_sample_fetch(accel_dev) < 0) {
        LOG_ERR("Error leyendo acelerómetro");
        return;
    }
   
    sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
    sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
    sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
   
    *x = sensor_value_to_double(&accel_x);
    *y = sensor_value_to_double(&accel_y);
    *z = sensor_value_to_double(&accel_z);
}

/* ==================== DISPLAY UPDATE ==================== */
static void update_display(void) {
    static float current_z = 0;
    uint32_t current_time = k_uptime_get();
   
    if (calibrating) {
        // No hacer nada mientras se muestra "CALIBRANDO"
        return;
    }
   
    if (!calibrated) {
        // Punto parpadeando (500ms encendido, 500ms apagado)
        if (current_time - last_blink_time > 500) {
            blink_state = !blink_state;
            last_blink_time = current_time;
           
            if (blink_state) {
                mb_display_image(display, MB_DISPLAY_MODE_SINGLE,
                                100, &IMG_CALIBRATE, 1);
            } else {
                mb_display_image(display, MB_DISPLAY_MODE_SINGLE,
                                100, &IMG_EMPTY, 1);
            }
        }
        return;
    }
   
    // Leer acelerómetro si está calibrado
    float x, y, z;
    read_accelerometer(&x, &y, &z);
    current_z = z;
   
    float difference = current_z - reference_z;
   
    if (fabsf(difference) < THRESHOLD) {
        // Postura correcta - mostrar dos líneas verticales (II) fijas
        mb_display_image(display, MB_DISPLAY_MODE_SINGLE,
                        500, &IMG_CORRECT, 1);
        alert_blinking = false;
    } else {
        // Postura incorrecta - mostrar alerta parpadeante
        alert_blinking = true;
       
        // Control del parpadeo de alerta (300ms encendido, 300ms apagado)
        if (current_time - last_alert_blink_time > 300) {
            alert_blink_state = !alert_blink_state;
            last_alert_blink_time = current_time;
        }
       
        if (alert_blink_state) {
            if (difference > THRESHOLD) {
                // Inclinado hacia adelante - mostrar <>
                mb_display_image(display, MB_DISPLAY_MODE_SINGLE,
                                100, &IMG_FORWARD, 1);
            } else {
                // Inclinado hacia atrás - mostrar ><
                mb_display_image(display, MB_DISPLAY_MODE_SINGLE,
                                100, &IMG_BACKWARD, 1);
            }
        } else {
            // Apagar durante la fase de parpadeo
            mb_display_image(display, MB_DISPLAY_MODE_SINGLE,
                            100, &IMG_EMPTY, 1);
        }
    }
}

/* ==================== BUTTON HANDLER (CALIBRATION) ==================== */
static void button_a_pressed(const struct device *dev,
                           struct gpio_callback *cb, uint32_t pins) {
    float x, y, z;
   
    if (calibrating) {
        return; // Evitar múltiples calibraciones simultáneas
    }
   
    calibrating = true;
   
    // Mostrar "CALIBRANDO" en scroll
    mb_display_print(display, MB_DISPLAY_MODE_SCROLL, 500, "CALIBRANDO");
   
    // Esperar a que termine el scroll
    k_msleep(3000); // Tiempo aproximado para el scroll
   
    // Leer acelerómetro para obtener referencia
    read_accelerometer(&x, &y, &z);
    reference_z = z;
    calibrated = true;
    calibrating = false;
    alert_blinking = false;
   
    LOG_INF("Calibrado! Referencia Z: %.2f, Umbral: %.2f", reference_z, THRESHOLD);
   
    // Pequeño delay para evitar rebotes
    k_msleep(50);
}

/* ==================== TEMPERATURE NOTIFICATION ==================== */
static void send_temperature_notification(void) {
    struct sensor_value temp_val;
    int ret;
   
    if (!device_is_ready(temp_dev)) {
        return;
    }
   
    ret = sensor_sample_fetch(temp_dev);
    if (ret < 0) {
        LOG_ERR("Error leyendo temperatura: %d", ret);
        return;
    }
   
    ret = sensor_channel_get(temp_dev, SENSOR_CHAN_DIE_TEMP, &temp_val);
    if (ret < 0) {
        LOG_ERR("Error obteniendo temperatura: %d", ret);
        return;
    }
   
    // Convertir a grados Celsius
    temperature_value = (int8_t)temp_val.val1;
   
    LOG_INF("Temperatura: %d°C", temperature_value);
   
    if (current_conn) {
        ret = bt_gatt_notify(current_conn,
                           &posture_svc.attrs[2],
                           &temperature_value,
                           sizeof(temperature_value));
        if (ret) {
            LOG_ERR("Error notificando temperatura: %d", ret);
        }
    }
}
