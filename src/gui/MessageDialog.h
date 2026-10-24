#pragma once

#include <functional>
#include <optional>
#include <string>

namespace slstudio::gui {
class MessageDialog {
public:
  void request(std::string title, std::string question, std::string confirmLabel,
      std::string details = {}, std::string icon = {});
  void draw(const std::function<void()> &confirm);

private:
  struct Message {
    std::string title;
    std::string question;
    std::string confirmLabel;
    std::string details;
    std::string icon;
  };

  std::optional<Message> message_;
  bool opened_ = false;
};
}
