namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<class CONTEXT>
typename Option<CONTEXT>::ResultType Option<CONTEXT>::execute(CONTEXT* context)
{
    for (auto child : Composite<CONTEXT>::children_)
    {
        ResultType childResult = child->execute(context);
        if (childResult != ResultType::FAILED)
        {
            return childResult;
        }
    }

    return ResultType::FAILED;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
