#include <iostream>
#include <string>
#include "etc/etc_provider.h"
#include "etc/etc_data.h"
#include "etc/etc_subject.h"
#include "etc/etc_observer.h"

using namespace comwise::etc;

int main()
{
    printf("hello etc ...\n");

    auto obs1 = std::make_shared<etc_observer>("obs1");
    auto obs2 = std::make_shared<etc_observer>("obs2");
    etc_subject sub;
    sub.register_observer(obs1);
    sub.register_observer(obs2);
    auto ptr = std::make_shared<etc_arg_t>();
    //auto param = etc_default::get_default();
    //ptr->set_param(std::move(param));
    sub.set_data(0, ptr);

    getchar();

    printf("exit etc ...\n");

    return 0;
}