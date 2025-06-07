/* Copyright Vital Audio, LLC
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#if VISAGE_MAC
#include "popup_menu.h"

#include <Cocoa/Cocoa.h>

@interface PopupMenuTarget : NSObject
- (void)invoke:(id)sender;
@end

@implementation PopupMenuTarget {
}

- (void)invoke:(id)sender {
  const visage::PopupMenu* menu = (const visage::PopupMenu*)[[sender representedObject] pointerValue];
  int id = menu ? menu->id() : 0;
  while (menu) {
    menu->onSelection().callback(id);
    menu = menu->parent();
  }
}
@end

namespace visage {
  class NativeMenu {
  public:
    static NativeMenu& instance() {
      static NativeMenu native_menu;
      return native_menu;
    }

    static const PopupMenu& menu() { return instance().menu_; }
    static void setPopup(const PopupMenu& menu) { instance().menu_ = menu; }

  private:
    PopupMenu menu_;
  };

  NSMenu* buildMenu(const PopupMenu& popup_menu, id target) {
    NSMenu* menu = [[NSMenu alloc] init];
    auto handler = [[PopupMenuTarget alloc] init];

    for (const auto& item : popup_menu.options()) {
      NSString* name = [NSString stringWithUTF8String:item.name().toUtf8().c_str()];

      if (item.isBreak())
        [menu addItem:[NSMenuItem separatorItem]];
      else if (!item.options().empty()) {
        NSMenuItem* submenu_item = [[NSMenuItem alloc] initWithTitle:name
                                                              action:nil
                                                       keyEquivalent:@""];

        [submenu_item setSubmenu:buildMenu(item, target)];
        [menu addItem:submenu_item];
      }
      else {
        NSMenuItem* menu_item = [[NSMenuItem alloc] initWithTitle:name
                                                           action:@selector(invoke:)
                                                    keyEquivalent:@""];
        if (item.isActive()) {
          [menu_item setTarget:handler];
          [menu_item setRepresentedObject:[NSValue valueWithPointer:&item]];
        }
        if (item.isSelected())
          [menu_item setState:NSControlStateValueOn];
        
        [menu addItem:menu_item];
      }
    }
    return menu;
  }

  void setNativeMenuBar(const PopupMenu& popup_menu) {
    @autoreleasepool {
      NativeMenu::setPopup(popup_menu);
      [NSApp setMainMenu:buildMenu(NativeMenu::menu(), nil)];
    }
  }
}

#endif
