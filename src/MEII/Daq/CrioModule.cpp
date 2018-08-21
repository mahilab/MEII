namespace meii {

Module::Module(
{}

bool MyRio::Connector::on_enable() {
    if (AI.enable() && AO.enable() && DIO.enable())
        return true;
    else
        return false;
}

bool MyRio::Connector::on_disable() {
    if (AI.disable() && AO.disable() && DIO.disable())
        return true;
    else
        return false;
}


}
