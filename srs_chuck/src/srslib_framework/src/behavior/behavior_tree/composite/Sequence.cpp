namespace srs {

////////////////////////////////////////////////////////////////////////////////////////////////////
// Public methods

////////////////////////////////////////////////////////////////////////////////////////////////////
template<class CONTEXT>
typename TreeNode<CONTEXT>::NodeResult Sequence<CONTEXT>::execute(CONTEXT* context)
{
    return TreeNode<CONTEXT>::NodeResult::SUCCEDED;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
// Private methods

} // namespace srs
