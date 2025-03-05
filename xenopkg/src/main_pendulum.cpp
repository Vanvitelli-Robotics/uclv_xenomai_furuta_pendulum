#include <rclcpp/rclcpp.hpp>

#include <std_srvs/srv/set_bool.hpp>
#include <xenopkg_interfaces/srv/joint_srv.hpp>
#include <xenopkg_interfaces/srv/set_mode.hpp>
#include <rclcpp/wait_for_message.hpp>
#include <xenopkg_interfaces/msg/float32_header.hpp>

float theta = 1.0;
float omega = 1.0;

void topic_callback(const xenopkg_interfaces::msg::Float32Header::SharedPtr message)
{
  theta = message->data[0];
  omega = message->data[1];
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("main_pendulum");

  // service client
  auto setserial = node->create_client<std_srvs::srv::SetBool>("set_serial_srv");
  auto setlqr = node->create_client<std_srvs::srv::SetBool>("activate_lqr_srv");
  auto setmode = node->create_client<xenopkg_interfaces::srv::SetMode>("set_mode_srv");
  auto movejoint = node->create_client<xenopkg_interfaces::srv::JointSrv>("joint_position_srv");
  auto subscription = node->create_subscription<xenopkg_interfaces::msg::Float32Header>("/encoder_data", 1, topic_callback);
  // Server UP
  RCLCPP_INFO(node->get_logger(), "Waiting for servers...");

  setserial->wait_for_service();
  setlqr->wait_for_service();
  setmode->wait_for_service();
  movejoint->wait_for_service();
  RCLCPP_INFO(node->get_logger(), "Servers UP");
  sleep(5);
  // porto il robot alla posizione desiderata qf
  {
    RCLCPP_INFO_STREAM(node->get_logger(), " moving to qf... ");

    auto movejoint_request = std::make_shared<xenopkg_interfaces::srv::JointSrv::Request>();
    movejoint_request->qf = {-0.75, 1.3962634016, -1.3962634016, 0.0, 0.0, M_PI_2};

    // chiamo il servizio e aspetto la risposta
    auto future_result = movejoint->async_send_request(movejoint_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      return -1;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,robot in qf");
    // check del successo del servizio
    if (!future_result.get()->completed)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return -1;
    }
  }

  // attivo la seriale aspettando che il pendolo sia fermo
  sleep(2);
  {

    RCLCPP_INFO_STREAM(node->get_logger(), "encoder activating...");

    auto set_serial_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    set_serial_request->data = true;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setserial->async_send_request(set_serial_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      return -1;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo

    // check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return -1;
    }

    // Variabili per la gestione del tempo

    rclcpp::Time timeout(5, 0);
    rclcpp::Time last_zero_time = node->now();
    // Attendere il primo messaggio
    RCLCPP_INFO(node->get_logger(), "calibration...");
    bool velwaszero = 0;

    rclcpp::Rate rate(1000);
    while (rclcpp::ok())
    {
      rate.sleep();
      // rclcpp::wait_for_message<xenopkg_interfaces::msg::Float32Header>(msg, node, "/encoder_data", std::chrono::milliseconds(1));
      rclcpp::spin_some(node);
      // printf("omega:%f",omega);
      if (velwaszero)
      {
        if (fabs(omega) > 1e-3)
        {
          velwaszero = 0;
          continue;
        }
        if ((node->now().seconds() - last_zero_time.seconds()) > timeout.seconds())
        {
          break;
        }
      }
      else
      {

        if (fabs(omega) < 1e-3)
        {
          // Se il valore cambia da 0, resetta il timer
          velwaszero = 1;
          RCLCPP_INFO(rclcpp::get_logger("rclcpp"), " timer reset");
          last_zero_time = node->now(); // Reset
        }
      }
    }

    // adesso devo resettare lo zero dell'encoder

    set_serial_request->data = false;

    // chiamo il servizio e aspetto la risposta
    auto future_result1 = setserial->async_send_request(set_serial_request);
    if (rclcpp::spin_until_future_complete(node, future_result1) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      return -1;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo

    // check del successo del servizio
    if (!future_result1.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return -1;
    }

    // rimando un true per far partire il publish dei valori corretti
    set_serial_request->data = true;

    // chiamo il servizio e aspetto la risposta
    auto future_result2 = setserial->async_send_request(set_serial_request);
    if (rclcpp::spin_until_future_complete(node, future_result2) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      return -1;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo

    // check del successo del servizio
    if (!future_result2.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return -1;
    }
  }

  // adesso metto il robot il velocity mode
  sleep(2);
  {
    RCLCPP_INFO_STREAM(node->get_logger(), " setting velocity mode... ");

    auto setmode_request = std::make_shared<xenopkg_interfaces::srv::SetMode::Request>();
    setmode_request->mode = 2;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setmode->async_send_request(setmode_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      return -1;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,velocity mode");
    // check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return -1;
    }
  }

  // attivo lqr
  sleep(2);
  {
    RCLCPP_INFO_STREAM(node->get_logger(), " activating lqr... ");

    auto setlqr_request = std::make_shared<std_srvs::srv::SetBool::Request>();
    setlqr_request->data = true;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setlqr->async_send_request(setlqr_request);
    if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
      return -1;
    }
    // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,lqr activate");
    // check del successo del servizio
    if (!future_result.get()->success)
    {
      RCLCPP_ERROR_STREAM(node->get_logger(),
                          "ERROR, SERVICE NOT SUCCESSFUL... message: ");
      return -1;
    }
  }

  rclcpp::Rate rate(1000);
  while (rclcpp::ok())
  {
    rate.sleep();
  }

  {
    RCLCPP_INFO_STREAM(node->get_logger(), " killing control,setting position mode... ");

    auto setmode_request = std::make_shared<xenopkg_interfaces::srv::SetMode::Request>();
    setmode_request->mode = 1;

    // chiamo il servizio e aspetto la risposta
    auto future_result = setmode->async_send_request(setmode_request);
    // if (rclcpp::spin_until_future_complete(node, future_result) != rclcpp::FutureReturnCode::SUCCESS)
    // {
    //   RCLCPP_ERROR_STREAM(node->get_logger(), "ERROR, SERVICE RESULT NOT AVAILABLE...");
    //   return -1;
    // }
    // // se arrivo quì il servizio è terminato, controllo se è terminato con successo
    // RCLCPP_INFO_STREAM(node->get_logger(), " request completed ,position mode");
    // // check del successo del servizio
    // if (!future_result.get()->success)
    // {
    //   RCLCPP_ERROR_STREAM(node->get_logger(),
    //                       "ERROR, SERVICE NOT SUCCESSFUL... message: ");
    //   return -1;
    // }
  }

  rclcpp::shutdown();
  return 0;
}