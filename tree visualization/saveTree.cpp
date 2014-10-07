
void ompl::geometric::FMTHeuristic::saveTree(const std::string &filename)
{
    std::ofstream ofs;
    ofs.open(filename.c_str(), std::ofstream::trunc);

    std::vector<Motion *> motions;
    nn_->list(motions);

    for (size_t i = 0; i < motions.size(); ++i)
    {
        if (motions[i]->getParent())
            ofs << motions[i]->getState()->as<base::RealVectorStateSpace::StateType>()->values[0] << "\t"
                << motions[i]->getState()->as<base::RealVectorStateSpace::StateType>()->values[1] << "\t"
                << motions[i]->getParent()->getState()->as<base::RealVectorStateSpace::StateType>()->values[0] << "\t"
                << motions[i]->getParent()->getState()->as<base::RealVectorStateSpace::StateType>()->values[1] << "\t"
                << motions[i]->getCost() << "\t" << motions[i]->getHeuristicCost() << std::endl << "\t"
                << opt_->combineCosts(motions[i]->getCost(), motions[i]->getHeuristicCost());
        else
            ofs << motions[i]->getState()->as<base::RealVectorStateSpace::StateType>()->values[0] << "\t"
                << motions[i]->getState()->as<base::RealVectorStateSpace::StateType>()->values[1] << "\t"
                << motions[i]->getState()->as<base::RealVectorStateSpace::StateType>()->values[0] << "\t"
                << motions[i]->getState()->as<base::RealVectorStateSpace::StateType>()->values[1] << "\t"
                << 0 << opt_->motionCostHeuristic(motions[i]->getState(), goalState_) << "\t"
                << opt_->motionCostHeuristic(motions[i]->getState(), goalState_) << std::endl;
    }

    ofs.close();
}
